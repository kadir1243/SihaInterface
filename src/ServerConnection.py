import requests
from PySide6.QtCore import qDebug, QTime
from requests import Response

# FIXME: This should be set to false
INTERNAL_DEBUG_SHOULD_BE_DISABLED_IN_PROD_BOOL_THAT_ENABLES_MY_OWN_SERVER_REPLICA_FOR_TEST = True
AUTH_UUID_TOKEN: str | None = None

def login_to_server(target_address: str, username: str, password: str) -> int:
    if INTERNAL_DEBUG_SHOULD_BE_DISABLED_IN_PROD_BOOL_THAT_ENABLES_MY_OWN_SERVER_REPLICA_FOR_TEST:
        requests.post(target_address + "/internal/add_new_user", json={"username": username, "password": password}, headers={"Content-Type": "application/json"})
    login = requests.post(target_address + "/api/giris", data=f'{{\"kadi\": \"{username}\",\"sifre\":\"{password}\"}}', headers={'Content-Type': 'application/json'})
    login.raise_for_status()
    if login.headers["uuid-token"]:
        global AUTH_UUID_TOKEN
        AUTH_UUID_TOKEN = login.headers["uuid-token"]
    else:
        AUTH_UUID_TOKEN = None
    return int(login.text)

class GpsSaati:
    saat: int
    dakika: int
    saniye: int
    milisaniye: int

    def __init__(self, time: QTime):
        self.milisaniye = time.msec()
        self.saniye = time.second()
        self.dakika = time.minute()
        self.saat = time.hour()
    def __str__(self) -> str:
        return "%s:%s:%s.%s" % (self.saat, self.dakika, self.dakika, self.milisaniye)

class TelemetryData:
    takim_numarasi: int
    iha_enlem: float
    iha_boylam: float
    iha_irtifa: float
    iha_dikilme: float
    iha_yonelme: float
    iha_yatis: float
    iha_hiz: float
    iha_batarya: float
    iha_otonom: int
    iha_kilitlenme: int
    hedef_merkez_X: int
    hedef_merkez_Y: int
    hedef_genislik: int
    hedef_yukseklik: int
    gps_saati: GpsSaati

    def __init__(self):
        self.takim_numarasi = -1
        self.iha_enlem = 0
        self.iha_boylam = 0
        self.iha_irtifa = 0
        self.iha_dikilme = 0
        self.iha_yonelme = 0
        self.iha_yatis = 0
        self.iha_hiz = 0
        self.iha_batarya = 0
        self.iha_otonom = 1
        self.iha_kilitlenme = 0
        self.hedef_merkez_X = 0
        self.hedef_merkez_Y = 0
        self.hedef_genislik = 0
        self.hedef_yukseklik = 0
        self.gps_saati = GpsSaati(QTime())

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
    sunucusaati: GpsSaati # I don't really care 'gun' field in response
    konumBilgileri: list[TelemetryResponseUavData]
    def __init__(self):
        self.sunucusaati = GpsSaati(QTime())
        self.konumBilgileri = []

SERVER_IS_UNREACHABLE_COUNTER: int = 0 # When this hits 100, disconnect from server

def send_telemetry(target_address: str, telemetry_data: TelemetryData) -> TelemetryResponseData:
    try:
        headers = {"Content-Type": "application/json"}
        if AUTH_UUID_TOKEN:
            headers["uuid-token"] = AUTH_UUID_TOKEN
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
        }, headers=headers, timeout=100)
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
    headers = {"Content-Type": "application/json"}
    if AUTH_UUID_TOKEN:
        headers["uuid-token"] = AUTH_UUID_TOKEN
    r: Response = requests.get(target_address + "/api/qr_koordinati", headers=headers)
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
    headers = {"Content-Type": "application/json"}
    if AUTH_UUID_TOKEN:
        headers["uuid-token"] = AUTH_UUID_TOKEN
    r: Response = requests.get(target_address + "/api/hss_koordinatlari", headers=headers)
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

def send_kamikaze(target_address: str, start: GpsSaati, end: GpsSaati, qr_text: str) -> None:
    headers = {"Content-Type": "application/json"}
    if AUTH_UUID_TOKEN:
        headers["uuid-token"] = AUTH_UUID_TOKEN
    r: Response = requests.post(target_address + "/api/kamikaze_bilgisi", json={
        "kamikazeBaslangicZamani": {
                "saat": start.saat,
                "dakika": start.dakika,
                "saniye": start.saniye,
                "milisaniye": start.milisaniye
        },
        "kamikazeBitisZamani": {
            "saat": end.saat,
            "dakika": end.dakika,
            "saniye": end.saniye,
            "milisaniye": end.milisaniye
        },
        "qrMetni": qr_text
    }, headers=headers)
    r.raise_for_status()

def send_kilitlenme(target_address: str, end: GpsSaati, automatic: bool) -> None:
    headers = {"Content-Type": "application/json"}
    if AUTH_UUID_TOKEN:
        headers["uuid-token"] = AUTH_UUID_TOKEN
    r: Response = requests.post(target_address + "/api/kilitlenme_bilgisi", json={
        "kilitlenmeBitisZamani": {
                "saat": end.saat,
                "dakika": end.dakika,
                "saniye": end.saniye,
                "milisaniye": end.milisaniye
        },
        "otonom_kilitlenme": 1 if automatic else 0
    }, headers=headers)
    r.raise_for_status()
