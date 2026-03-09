import struct

import lz4.block
from PySide6.QtCore import QTimer, qWarning, qInfo
from PySide6.QtGui import QImage
from PySide6.QtMultimedia import QVideoFrame
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtNetwork import QUdpSocket, QHostAddress, QAbstractSocket
from PySide6.QtWidgets import QWidget


class CameraWidget(QVideoWidget):
    reconnect_timer: QTimer
    udpSocket: QUdpSocket

    def __init__(self, parent: QWidget | None = None):
        QVideoWidget.__init__(self, parent=parent)
        self.reconnect_timer = QTimer()
        self.udpSocket = QUdpSocket(self)
        self.init_socket()

    def error_happened_in_socket(self, error: QAbstractSocket.SocketError):
        qWarning(self.udpSocket.errorString())
        qWarning(str(error.value))

    def init_socket(self):
        self.udpSocket.bind(QHostAddress("127.0.0.1"), port=8000)
        self.udpSocket.readyRead.connect(self.readPendingDatagrams)
        self.udpSocket.errorOccurred.connect(self.error_happened_in_socket)

    last_index_of_frame: int = 0
    last_amount_of_parts: int = 0
    frame_data_map: dict[int, bytes] = {}
    amount_of_parts_received: int = 0

    # My Internal Server Code
    #def split_bylen(item, maxlen):  # I don't remember where i copied this from, but thanks for answer in stackoverflow :D
    #    '''
    #    Requires item to be sliceable (with __getitem__ defined)
    #    '''
    #    return [item[ind:ind + maxlen] for ind in range(0, len(item), maxlen)]
    ## Reminder for myself
    ## header                                          Binary Frame Data
    ## [4 Byte Integer][4 Byte Integer] [4 Byte Integer][4 Byte Integer][Byte Array]
    ## [Part of frame ][Amount Of Parts][Index of frame][Size of Frame ][Data]
    #def start_server():
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
    #            sock.sendto(struct.pack("!IIII", j + 1, length, i, size) + e, address)
    #            time.sleep(1 / 100000)
    #        i = i + 1
    #if __name__ == '__main__':
    #    start_server()

    def readPendingDatagrams(self):
        while self.udpSocket.hasPendingDatagrams():
            datagram = self.udpSocket.receiveDatagram()
            data = datagram.data()
            header = struct.unpack("!IIII", data[:16])
            part_of_frame: int = header[0]
            amount_of_parts: int = header[1]
            index_of_frame: int = header[2]
            size_in_bytes: int = header[3]
            if index_of_frame < self.last_index_of_frame:
                qInfo("Old frame received, Skipping frame")
                continue # there is a missing frame, then we can just skip this frame
            elif index_of_frame > self.last_index_of_frame:
                if self.amount_of_parts_received != self.last_amount_of_parts:
                    qInfo("Received new frame before finishing older one")
                self.frame_data_map.clear()
                self.amount_of_parts_received = 0

            self.last_index_of_frame = index_of_frame
            self.last_amount_of_parts = amount_of_parts
            if part_of_frame in self.frame_data_map:
                qWarning("Duplicate packet??")
                continue
            if part_of_frame > amount_of_parts or size_in_bytes < 0:
                qWarning("Broken packet maybe??")
                continue
            self.frame_data_map[part_of_frame] = data[16:]
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
                self.videoSink().setVideoFrame(QVideoFrame(img))
                self.frame_data_map.clear()
                self.amount_of_parts_received = 0


    def start_reconnect_timer(self):
        self.reconnect_timer.start()

    def stop_reconnect_timer(self):
        self.reconnect_timer.stop()
