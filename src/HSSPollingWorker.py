"""
HSSPollingWorker — Otomatik HSS (Hava Savunma Sistemi) Veri Çekici

Sunucu bağlantısı kurulduğunda QThread üzerinde başlar.
Yapılandırılabilir aralıklarla (varsayılan 3sn) /api/hss_koordinatlari'nı çeker.
Yalnızca liste değiştiğinde hss_updated sinyali yayar.
"""

from PySide6.QtCore import QObject, Signal, qDebug, qWarning, QTimer

from src.CommonUtils import HssSnapshot
from src.ServerConnection import get_ads, ServerAdsData


class HSSPollingWorker(QObject):
    hss_updated = Signal(HssSnapshot)
    hss_error = Signal(str)
    timer: QTimer

    def __init__(self, server_address: str, interval_ms: int = 3000):
        super().__init__()
        self.server_address = server_address
        self.running = False
        self._seq: int = 0
        self._last_zones: list[ServerAdsData] = []
        qDebug("HSS polling timer configured to %s ms" % interval_ms)
        self.timer = QTimer(self, interval=interval_ms)
        self.timer.timeout.connect(self._poll)

    def run(self) -> None:
        self.running = True
        self.timer.start()

    def stop(self) -> None:
        self.running = False
        self.timer.stop()

    def _poll(self) -> None:
        try:
            result = get_ads(self.server_address)
            if result is None:
                self.hss_error.emit("HSS verisi alınamadı")
                return

            if tuple(result) != tuple(self._last_zones):
                self._seq += 1
                self._last_zones = result
                snapshot = HssSnapshot(
                    seq=self._seq,
                    zones=result
                )
                self.hss_updated.emit(snapshot)
        except Exception as e:
            err = "HSS polling beklenmeyen hata: %s" % str(e)
            qWarning(err)
            self.hss_error.emit(err)
