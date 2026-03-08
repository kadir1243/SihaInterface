import requests
from PySide6.QtCore import qDebug
from requests import Response

# FIXME: This should be set to false
INTERNAL_DEBUG_SHOULD_BE_DISABLED_IN_PROD_BOOL_THAT_ENABLES_MY_OWN_SERVER_REPLICA_FOR_TEST = True

def login_to_server(target_address: str, username: str, password: str) -> int:
    if not target_address.startswith("http"):
        target_address = "http://" + target_address
    if INTERNAL_DEBUG_SHOULD_BE_DISABLED_IN_PROD_BOOL_THAT_ENABLES_MY_OWN_SERVER_REPLICA_FOR_TEST:
        requests.post(target_address + "/internal/add_new_user", json={"team": 1, "username": username, "password": password}, headers={"Content-Type": "application/json"})
    login = requests.post(target_address + "/api/giris", data=f'{{\"kadi\": \"{username}\",\"sifre\":\"{password}\"}}', headers={'Content-Type': 'application/json'})
    login.raise_for_status()
    return int(login.text)

class GpsSaati:
    saat: int = 0
    dakika: int = 0
    saniye: int = 0
    milisaniye: int = 0

    def __init__(self, msec: int):
        second = msec // 100
        self.milisaniye = second % 1
        minute = second // 60
        self.dakika = minute % 1
        hour = minute // 60
        self.saat = hour


class TelemetryData:
    takim_numarasi: int = -1
    iha_enlem: float = 0
    iha_boylam: float = 0
    iha_irtifa: float = 0
    iha_dikilme: float = 0
    iha_yonelme: float = 0
    iha_yatis: float = 0
    iha_hiz: float = 0
    iha_batarya: float = 0
    iha_otonom: int = 1
    iha_kilitlenme: int = 0
    hedef_merkez_X: int = 0
    hedef_merkez_Y: int = 0
    hedef_genislik: int = 0
    hedef_yukseklik: int = 0
    gps_saati: GpsSaati = GpsSaati(0)

class TelemetryResponseUavData:
    takim_numarasi: int
    iha_enlem: float
    iha_boylam: float
    iha_irtifa: float
    iha_dikilme: float
    iha_yonelme: int
    iha_yatis: float
    iha_hizi: float
    zaman_farki: int

class TelemetryResponseData:
    sunucusaati: GpsSaati = GpsSaati(0) # I don't really care 'gun' field in response
    konumBilgileri: list[TelemetryResponseUavData] = []

def send_telemetry(target_address: str, telemetry_data: TelemetryData) -> TelemetryResponseData:
    if not target_address.startswith("http"):
        target_address = "http://" + target_address
    r: Response = requests.post(target_address + "/api/telemetri_gonder", json={
        "takim_numarasi": telemetry_data.takim_numarasi,
        "iha_enlem": telemetry_data.iha_enlem,
        "iha_boylam": telemetry_data.iha_boylam,
        "iha_irtifa": telemetry_data.iha_irtifa,
        "iha_dikilme": telemetry_data.iha_dikilme,
        "iha_yonelme": telemetry_data.iha_yonelme,
        "iha_yatis": telemetry_data.iha_yatis,
        "iha_hiz": telemetry_data.iha_hiz,
        "iha_batarya": telemetry_data.iha_batarya,
        "iha_otonom": telemetry_data.iha_otonom,
        "iha_kilitlenme": telemetry_data.iha_kilitlenme,
        "hedef_merkez_X": telemetry_data.hedef_merkez_X,
        "hedef_merkez_Y": telemetry_data.hedef_merkez_Y,
        "hedef_genislik": telemetry_data.hedef_genislik,
        "hedef_yukseklik": telemetry_data.hedef_yukseklik,
        "gps_saati": {
            "saat": telemetry_data.gps_saati.saat,
            "dakika": telemetry_data.gps_saati.dakika,
            "saniye": telemetry_data.gps_saati.saniye,
            "milisaniye": telemetry_data.gps_saati.milisaniye
        }
    }, headers={"Content-Type": "application/json"}, timeout=100)
    r.raise_for_status()
    data = r.json()
    d: TelemetryResponseData = TelemetryResponseData()
    d.sunucusaati = data.get("sunucusaati")
    # TODO: I need to write
    return d


TELEMETRY_DATA: TelemetryData = TelemetryData()
SERVER_TELEMETRY_RESPONSE: TelemetryResponseData = TelemetryResponseData()

class QrCoords:
    qrEnlem: float
    qrBoylam: float

def get_kamikaze_coords(target_address: str) -> QrCoords:
    if not target_address.startswith("http"):
        target_address = "http://" + target_address
    r: Response = requests.get(target_address + "/api/qr_koordinati")
    r.raise_for_status()
    data = r.json()
    d: QrCoords = QrCoords()
    d.qrEnlem = data.get("qrEnlem")
    d.qrBoylam = data.get("qrBoylam")
    return d
