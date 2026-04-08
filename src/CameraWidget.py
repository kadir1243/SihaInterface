import struct
from enum import Enum

import lz4.block
from PySide6.QtCore import QTimer, qWarning, qInfo, QThread, Qt, qDebug, QObject, Signal, QSize
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtNetwork import QUdpSocket, QHostAddress, QAbstractSocket
from PySide6.QtWidgets import QWidget, QLabel, QGridLayout


class SocketWrapper(QObject):
    socket: QUdpSocket | None = None
    parentWidget: CameraWidget
    update_camera_in_ui: Signal = Signal(QPixmap)

    def __init__(self, parentWidget: CameraWidget):
        super().__init__()
        self.parentWidget = parentWidget
        self.update_camera_in_ui.connect(lambda img: self.parentWidget.label.setPixmap(img.scaled(self.parentWidget.size(), Qt.AspectRatioMode.KeepAspectRatio)))

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
    def bindSocket(self):
        if not self.socket:
            self.socket = QUdpSocket()
            self.socket.readyRead.connect(self.readPendingDatagrams)
            self.socket.errorOccurred.connect(self.error_happened_in_socket)
        self.socket.bind(QHostAddress("127.0.0.1"), port=8000)
        self.last_index_of_frame = 0
        self.last_amount_of_parts = 0
        self.frame_data_map = {}
        self.amount_of_parts_received = -1
        self.connected_server_id = 0

    def readPendingDatagrams(self):
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
                img = QImage(lz4.block.decompress(blob), 640, 480, QImage.Format.Format_RGB888)
                self.update_camera_in_ui.emit(QPixmap.fromImageInPlace(img))
                self.frame_data_map.clear()
                self.amount_of_parts_received = 0
    def error_happened_in_socket(self, error: QAbstractSocket.SocketError):
        qWarning("%s" % self.socket.errorString())
        qWarning("%s" % error.value)
    def close(self):
        self.socket.close()

class CameraServerProtocol(Enum):
    Osman = (0,)
    Kadir = (1,)
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

class CameraWidget(QWidget):
    reconnect_timer: QTimer
    connection_thread: QThread | None = None
    socketWorker: SocketWrapper | None = None
    camera_server_info: CameraServerInfo = CameraServerInfo()
    label: QLabel

    def __init__(self, parent: QWidget | None = None):
        QWidget.__init__(self, parent=parent)
        self.gridLayout = QGridLayout(self)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(self)
        self.label.setScaledContents(True)
        self.set_no_connection_image()
        self.label.show()
        self.gridLayout.addWidget(self.label)

        self.reconnect_timer = QTimer(self)

    def set_no_connection_image(self):
        self.label.setPixmap(QPixmap.fromImageInPlace(QImage("ui_files/no_video_stream_found.png")).scaled(QSize(500, 200)))

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
        self.socketWorker = SocketWrapper(self)
        self.connection_thread = QThread(self)
        self.connection_thread.setObjectName("Camera Connection Thread")
        self.connection_thread.finished.connect(self.socketWorker.close)
        self.connection_thread.started.connect(self.socketWorker.bindSocket)
        self.socketWorker.moveToThread(self.connection_thread)
        self.connection_thread.start()

    def closeSocket(self):
        if not self.connection_thread or not self.connection_thread.isRunning():
            self.connection_thread = None
            qDebug("Camera Connection Thread already closed")
            return
        self.connection_thread.quit()
        self.connection_thread.wait()
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
        match self.camera_server_info.protocol:
            case CameraServerProtocol.Osman:
                pass # TODO:
            case CameraServerProtocol.Kadir:
                self.closeSocket()

    def on_play(self):
        self.is_paused = False
        if not self.camera_server_info.ip:
            return False
        match self.camera_server_info.protocol:
            case CameraServerProtocol.Osman:
                pass # TODO:
            case CameraServerProtocol.Kadir:
                self.bindSocket()
        return True

    def connect_to_server(self) -> bool:
        if not self.camera_server_info.ip:
            return False
        match self.camera_server_info.protocol:
            case CameraServerProtocol.Osman:
                pass # TODO:
            case CameraServerProtocol.Kadir:
                self.bindSocket()
        return True

    def disconnect_from_server(self) -> bool:
        if not self.camera_server_info.ip:
            return False
        match self.camera_server_info.protocol:
            case CameraServerProtocol.Osman:
                pass # TODO:
            case CameraServerProtocol.Kadir:
                self.closeSocket()
        return True

    def set_protocol(self, i: int) -> None:
        self.camera_server_info.protocol = CameraServerProtocol.from_id(i)