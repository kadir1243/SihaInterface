import struct
from enum import Enum

import cv2
import lz4.block
import numpy
from PySide6.QtCore import QTimer, qWarning, qInfo, QThread, Qt, qDebug, QObject, Signal, QSize, QPointF, \
    QPoint, QProcess, QByteArray
from PySide6.QtGui import QImage, QPixmap, QPaintEvent, QPainterPath, QPainter, QPen
from PySide6.QtNetwork import QUdpSocket, QHostAddress, QAbstractSocket, QTcpSocket
from PySide6.QtWidgets import QWidget, QLabel, QGridLayout

from src.CommonUtils import KamikazeState
from src.barcode import RobustDetector, resolve_engine, draw_results

class AbstractProtocolWrapper(QObject):
    socket: QAbstractSocket | None
    parentWidget: CameraWidget
    update_camera_in_ui: Signal = Signal(bytes)
    def __init__(self, parentWidget: CameraWidget):
        super().__init__()
        self.parentWidget = parentWidget
        self.socket = None
        self._qr_engine = resolve_engine()
        self._qr_detector = RobustDetector(self._qr_engine)

    def bindSocket(self) -> None:
        pass

    def error_happened_in_socket(self, error: QAbstractSocket.SocketError) -> None:
        qWarning("%s" % self.socket.errorString())
        qWarning("%s" % error.value)

    def close(self) -> None:
        if self.socket:
            self.socket.close()

    def emit_camera_data(self, raw_image: bytes):
        width = self.parentWidget.camera_server_info.width
        height = self.parentWidget.camera_server_info.height

        frame = numpy.frombuffer(raw_image, dtype=numpy.uint8).copy().reshape((height, width, 3))
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        results = self._qr_detector.detect(gray)
        if results:
            draw_results(frame, results)
            for data, sym_type, pts in results:
                self.parentWidget.qr_successfully_readed.emit(data)

        self.update_camera_in_ui.emit(frame.tobytes())

class ProtocolKadirSocketWrapper(AbstractProtocolWrapper):
    def __init__(self, parentWidget: CameraWidget):
        super().__init__(parentWidget)

    # My Internal Server Code
    #def split_bylen(item, maxlen):  # I don't remember where i copied this from, but thanks for answer in stackoverflow :D
    #    '''
    #    Requires item to be sliceable (with __getitem__ defined)
    #    '''
    #    return [item[ind:ind + maxlen] for ind in range(0, len(item), maxlen)]
    ## Reminder for myself
    ## header                                          Binary Frame Data
    ## [4 Byte Integer][4 Byte Integer][4 Byte Integer][4 Byte Integer][1 Byte Integer][Byte Array]
    ## [Part of frame][Amount Of Parts][Index of frame][Size of Frame ][Random number ][Data]
    #def start_server():
    #    random_id = random.randint(0, 255)
    #    address = ('127.0.0.1', 8000)
    #    sock = socket.socket(type=socket.SOCK_DGRAM)
    #    webcam = Webcam(w=640, h=480)
    #    i: int = 0
    #    for frame in webcam:
    #        d = lz4.block.compress(frame.tobytes())
    #        size = len(d)
    #        split = split_bylen(d, 5000)
    #        length = len(split)
    #        for j, e in enumerate(split):
    #            sock.sendto(struct.pack("!IIIIB", j + 1, length, i, size, random_id) + e, address)
    #            time.sleep(1 / 100000)
    #        i = i + 1
    #if __name__ == '__main__':
    #    start_server()

    last_index_of_frame: int
    last_amount_of_parts: int
    frame_data_map: dict[int, bytes]
    amount_of_parts_received: int
    connected_server_id: int

    def bindSocket(self) -> None:
        if not self.socket:
            self.socket = QUdpSocket()
            self.socket.readyRead.connect(self.readPendingDatagrams)
            self.socket.errorOccurred.connect(self.error_happened_in_socket)
        self.socket.bind(QHostAddress(self.parentWidget.camera_server_info.ip), port=self.parentWidget.camera_server_info.port)
        self.last_index_of_frame = 0
        self.last_amount_of_parts = 0
        self.frame_data_map = {}
        self.amount_of_parts_received = -1
        self.connected_server_id = 0

    def readPendingDatagrams(self) -> None:
        while self.socket.hasPendingDatagrams():
            datagram = self.socket.receiveDatagram()
            data = datagram.data()
            header = struct.unpack("!IIIIB", data[:17])
            part_of_frame: int = header[0]
            amount_of_parts: int = header[1]
            index_of_frame: int = header[2]
            size_in_bytes: int = header[3]
            random_id: int = header[4]
            if self.connected_server_id != random_id:
                qInfo("New Camera Server Detected")
                self.connected_server_id = random_id
                self.last_index_of_frame = 0
                self.amount_of_parts_received = 0
                self.frame_data_map.clear()
            if index_of_frame < self.last_index_of_frame:
                qInfo("Old frame received, Skipping frame")
                continue # there is a missing frame, then we can just skip this frame
            elif index_of_frame > self.last_index_of_frame:
                if self.amount_of_parts_received != 0:
                    qInfo("Received new frame before finishing older one")
                    self.amount_of_parts_received = 0
                self.frame_data_map.clear()

            self.last_index_of_frame = index_of_frame
            self.last_amount_of_parts = amount_of_parts
            if part_of_frame in self.frame_data_map:
                qWarning("Duplicate packet??")
                continue
            if part_of_frame > amount_of_parts or size_in_bytes < 0:
                qWarning("Broken packet maybe??")
                continue
            self.frame_data_map[part_of_frame] = data[17:]
            self.amount_of_parts_received += 1
            if amount_of_parts == self.amount_of_parts_received:
                # avengers assemble
                blob = b''.join(self.frame_data_map[i] for i in sorted(self.frame_data_map))
                if len(blob) != size_in_bytes: # I added this to header because i don't trust my code
                    qWarning("Not enough bytes in frame")
                    self.frame_data_map.clear()
                    self.amount_of_parts_received = 0
                    continue
                self.emit_camera_data(lz4.block.decompress(blob))
                self.frame_data_map.clear()
                self.amount_of_parts_received = 0

class ProtocolOsmanSocketWrapper(AbstractProtocolWrapper):
    ffmpeg_process: QProcess | None = None
    frame_byte_size: int

    def __init__(self, parentWidget: CameraWidget):
        super().__init__(parentWidget)

    def bindSocket(self) -> None:
        if not self.socket:
            self.socket = QTcpSocket()
            self.socket.readyRead.connect(self._on_ready_read)
            self.socket.errorOccurred.connect(self.error_happened_in_socket)

        self.__frame_buffer = b""
        self.frame_byte_size = self.parentWidget.camera_server_info.frame_byte_size_buffer
        self.ffmpeg_process = QProcess(self)
        self.ffmpeg_process.setProgram("ffmpeg")
        self.ffmpeg_process.setArguments([
            "-hide_banner",
            "-loglevel", "warning",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-i", "pipe:0",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{self.parentWidget.camera_server_info.width}x{self.parentWidget.camera_server_info.height}",
            "pipe:1",
        ])
        self.ffmpeg_process.setReadChannel(QProcess.ProcessChannel.StandardOutput)
        self.ffmpeg_process.readyReadStandardOutput.connect(self._read_frames)
        self.ffmpeg_process.start()
        if not self.ffmpeg_process.waitForStarted():
            qWarning("Could not start ffmpeg process")

        self.socket.connectToHost(self.parentWidget.camera_server_info.ip, self.parentWidget.camera_server_info.port)

    __frame_buffer: bytes
    def _read_frames(self) -> None:
        self.__frame_buffer += self.ffmpeg_process.readAllStandardOutput().data()
        while len(self.__frame_buffer) >= self.frame_byte_size:
            raw = self.__frame_buffer[:self.frame_byte_size]
            self.__frame_buffer = self.__frame_buffer[self.frame_byte_size:]
            self.emit_camera_data(raw)

    def _on_ready_read(self) -> None:
        if not self.ffmpeg_process or self.ffmpeg_process.state() != QProcess.ProcessState.Running:
            return
        data: QByteArray = self.socket.readAll()
        self.ffmpeg_process.write(data)

    def close(self) -> None:
        if self.ffmpeg_process:
            self.ffmpeg_process.closeWriteChannel()
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.waitForFinished(2000)
            self.ffmpeg_process = None
        super().close()

class ProtocolBerkeSocketWrapper(AbstractProtocolWrapper):
    ffmpeg_process: QProcess | None = None
    frame_byte_size: int

    def __init__(self, parentWidget: CameraWidget):
        super().__init__(parentWidget)

    def bindSocket(self) -> None:
        self.__frame_buffer = b""
        self.frame_byte_size = self.parentWidget.camera_server_info.frame_byte_size_buffer
        self.ffmpeg_process = QProcess(self)
        self.ffmpeg_process.setProgram("ffmpeg")
        self.ffmpeg_process.setArguments([
            "-hide_banner",
            "-loglevel", "warning",
            "-fflags", "nobuffer+discardcorrupt+genpts",
            "-avioflags", "direct",
            "-err_detect", "ignore_err",
            "-i", f"udp://{self.parentWidget.camera_server_info.ip}:{self.parentWidget.camera_server_info.port}?timeout=2000000&reuse=1&buffer_size=65536",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{self.parentWidget.camera_server_info.width}x{self.parentWidget.camera_server_info.height}",
            "-fps_mode", "passthrough",
            "pipe:1",
        ])
        self.ffmpeg_process.setReadChannel(QProcess.ProcessChannel.StandardOutput)
        self.ffmpeg_process.readyReadStandardOutput.connect(self._read_frames)
        self.ffmpeg_process.errorOccurred.connect(self.process_error)
        self.ffmpeg_process.readyReadStandardError.connect(self._on_ready_standard_error_read)
        self.ffmpeg_process.start()
        if not self.ffmpeg_process.waitForStarted():
            qWarning("Could not start ffmpeg process")

    def _on_ready_standard_error_read(self) -> None:
        if not self.ffmpeg_process or self.ffmpeg_process.state() != QProcess.ProcessState.Running:
            return
        qWarning("ffmpeg error: %s" % self.ffmpeg_process.readAllStandardError().toStdString())

    def process_error(self, error: QProcess.ProcessError):
        qWarning("ffmpeg process failed with error: %s" % error)

    __frame_buffer: bytes
    def _read_frames(self) -> None:
        self.__frame_buffer += self.ffmpeg_process.readAllStandardOutput().data()
        while len(self.__frame_buffer) >= self.frame_byte_size:
            raw = self.__frame_buffer[:self.frame_byte_size]
            self.__frame_buffer = self.__frame_buffer[self.frame_byte_size:]
            self.emit_camera_data(raw)

    def close(self) -> None:
        if self.ffmpeg_process:
            self.ffmpeg_process.closeWriteChannel()
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.waitForFinished(2000)
            self.ffmpeg_process = None
        super().close()

class CameraServerProtocol(Enum):
    Berke = (0, "Berke")
    Osman = (1, "Osman")
    Kadir = (2, "Kadir")
    @staticmethod
    def from_id(i: int) -> CameraServerProtocol:
        e: CameraServerProtocol
        for e in CameraServerProtocol:
            if e.value[0] == i:
                return e
        return None

class CameraServerInfo:
    ip: str | None = None
    port: int
    protocol: CameraServerProtocol
    width: int
    height: int
    frame_byte_size_buffer: int
    def recalculate_frame_size(self) -> None:
        self.frame_byte_size_buffer = self.width * self.height * 3

class LabelWithRectangle(QLabel):
    is_no_stream_image: bool
    def __init__(self, parent):
        QLabel.__init__(self, parent=parent)
        self.is_no_stream_image = True

    def paintEvent(self, e: QPaintEvent, /):
        super().paintEvent(e)
        if self.is_no_stream_image:
            return
        size: QSize = self.size()
        pos: QPoint = self.pos()
        vertical_size = size.height() / 10
        horizontal_size = size.width() / 4
        painter: QPainter = QPainter(self)

        point1 = QPointF(pos.x() + size.width() - horizontal_size, pos.y() + size.height() - vertical_size)
        point2 = QPointF(pos.x() + horizontal_size, pos.y() + size.height() - vertical_size)
        point3 = QPointF(pos.x() + horizontal_size, pos.y() + vertical_size)
        point4 = QPointF(pos.x() + size.width() - horizontal_size, pos.y() + vertical_size)

        painter_path: QPainterPath = QPainterPath(point1)
        painter_path.lineTo(point2)
        painter_path.lineTo(point3)
        painter_path.lineTo(point4)
        painter_path.lineTo(point1)
        pen: QPen = QPen(Qt.GlobalColor.yellow)
        painter.setPen(pen)
        painter.drawPath(painter_path)

class CameraWidget(QWidget):
    reconnect_timer: QTimer
    connection_thread: QThread | None = None
    socketWorker: AbstractProtocolWrapper | None = None
    camera_server_info: CameraServerInfo = CameraServerInfo()
    qr_successfully_readed: Signal = Signal(str)
    label: LabelWithRectangle

    def __init__(self, parent: QWidget | None = None):
        QWidget.__init__(self, parent=parent)
        self.gridLayout = QGridLayout(self)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.label = LabelWithRectangle(self)
        self.label.setScaledContents(True)
        self.set_no_connection_image()
        self.label.show()
        self.gridLayout.addWidget(self.label)

        self.reconnect_timer = QTimer(parent=self, singleShot=True, interval=15000)
        self.reconnect_timer.timeout.connect(self.__fire_reconnect)

    def parent_is_lock_enabled(self) -> bool:
        return self.parentWidget().ui.disable_enable_locking.isChecked()

    def parent_is_kamikaze(self) -> bool:
        return self.parentWidget().kamikaze_state == KamikazeState.IDLE

    def __fire_reconnect(self):
        self.closeSocket()
        self.bindSocket()

    def set_no_connection_image(self):
        self.label.setPixmap(QPixmap.fromImageInPlace(QImage("ui_files/no_video_stream_found.png")).scaled(QSize(500, 200), Qt.AspectRatioMode.KeepAspectRatioByExpanding, Qt.TransformationMode.SmoothTransformation))
        self.label.is_no_stream_image = True

    def resizeEvent(self, event, /):
        if event.size().height() == 0 or event.size().width() == 0: # Widget should be hidden, for some reason hide and show does not get called on qsplitter
            if self.connection_thread:
                self.disconnect_from_server()
        else:
            if not self.connection_thread:
                self.connect_to_server()
        super().resizeEvent(event)

    def hideEvent(self, event, /):
        super().hideEvent(event)
        self.disconnect_from_server()

    def showEvent(self, event, /):
        super().showEvent(event)
        self.connect_to_server()

    def bindSocket(self):
        if self.connection_thread:
            if not self.connection_thread.isRunning():
                qWarning("Camera thread is not running but thought as running? Trying to restart thread")
                self.closeSocket()
                self.bindSocket()
                return
            qDebug("Camera Connection Thread already started")
            return
        self.label.is_no_stream_image = False
        if self.camera_server_info.protocol == CameraServerProtocol.Kadir:
            self.socketWorker = ProtocolKadirSocketWrapper(self)
        elif self.camera_server_info.protocol == CameraServerProtocol.Osman:
            self.socketWorker = ProtocolOsmanSocketWrapper(self)
        else:
            self.socketWorker = ProtocolBerkeSocketWrapper(self)
        self.socketWorker.update_camera_in_ui.connect(self.update_camera_img)
        self.connection_thread = QThread(self)
        self.connection_thread.setObjectName("Camera Connection Thread")
        self.connection_thread.finished.connect(self.socketWorker.close, Qt.ConnectionType.DirectConnection)
        self.connection_thread.started.connect(self.socketWorker.bindSocket, Qt.ConnectionType.DirectConnection)
        self.socketWorker.moveToThread(self.connection_thread)
        self.connection_thread.start()
        self.start_reconnect_timer()

    def update_camera_img(self, img: bytes) -> None:
        self.start_reconnect_timer()
        self.label.setPixmap(QPixmap.fromImage(QImage(img, self.camera_server_info.width, self.camera_server_info.height, QImage.Format.Format_RGB888).scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio)))

    def closeSocket(self):
        if not self.connection_thread or not self.connection_thread.isRunning():
            self.connection_thread = None
            qDebug("Camera Connection Thread already closed")
            return
        self.stop_reconnect_timer()
        self.connection_thread.quit()
        if not self.connection_thread.wait(2000):
            self.connection_thread.terminate()
        self.connection_thread = None
        self.socketWorker = None
        qDebug("Closed Camera Connection Thread")

    def start_reconnect_timer(self):
        self.reconnect_timer.start()

    def stop_reconnect_timer(self):
        self.reconnect_timer.stop()

    is_paused: bool = False

    def on_pause(self):
        self.is_paused = True
        if not self.camera_server_info.ip:
            return False
        self.closeSocket()
        return True

    def on_play(self):
        self.is_paused = False
        if not self.camera_server_info.ip:
            return False
        self.bindSocket()
        return True

    def connect_to_server(self) -> bool:
        if not self.camera_server_info.ip:
            return False
        self.bindSocket()
        return True

    def disconnect_from_server(self) -> bool:
        if not self.camera_server_info.ip:
            return False
        self.closeSocket()
        return True

    def set_protocol(self, i: int) -> None:
        self.camera_server_info.protocol = CameraServerProtocol.from_id(i)