import requests
from PySide6.QtCore import qDebug, QTime
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

    def __init__(self, time: QTime):
        self.milisaniye = time.msec()
        self.saniye = time.second()
        self.dakika = time.minute()
        self.saat = time.hour()

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
    gps_saati: GpsSaati = GpsSaati(QTime())

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
    sunucusaati: GpsSaati = GpsSaati(QTime()) # I don't really care 'gun' field in response
    konumBilgileri: list[TelemetryResponseUavData] = []

SERVER_IS_UNREACHABLE_COUNTER: int = 0 # When this hits 100, disconnect from server

def send_telemetry(target_address: str, telemetry_data: TelemetryData) -> TelemetryResponseData:
    if not target_address.startswith("http"):
        target_address = "http://" + target_address
    try:
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
        uav_s = list()
        for s in data["konumBilgileri"]:
            data: TelemetryResponseUavData = TelemetryResponseUavData()
            data.takim_numarasi = s["takim_numarasi"]
            data.iha_enlem = s["iha_enlem"]
            data.iha_boylam = s["iha_boylam"]
            data.iha_yatis = s["iha_yatis"]
            uav_s.append(data)
        d.konumBilgileri = uav_s
        # FIXME: I don't need any other data for now, i will add it when i needed
        return d
    except ConnectionError as e:
        global SERVER_IS_UNREACHABLE_COUNTER
        SERVER_IS_UNREACHABLE_COUNTER += 1

        return None

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

class ServerAdsData:
    id: int
    hssEnlem: float
    hssBoylam: float
    hssYariCap: float

def get_ads(target_address: str) -> list[ServerAdsData]:
    if not target_address.startswith("http"):
        target_address = "http://" + target_address
    r: Response = requests.get(target_address + "/api/hss_koordinatlari")
    r.raise_for_status()
    data = r.json()

    ads_list: list[ServerAdsData] = list()
    for d in data["hss_koordinat_bilgileri"]:
        data: ServerAdsData = ServerAdsData()
        data.id = d["id"]
        data.hssEnlem = d["hssEnlem"]
        data.hssBoylam = d["hssBoylam"]
        data.hssYariCap = d["hssYaricap"]
        ads_list.append(data)
    return ads_list
