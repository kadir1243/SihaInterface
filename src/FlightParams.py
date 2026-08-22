"""
Uçuş zarfı sabitleri ve parametre tabloları.

Sabitler, onları araca yazan BASELINE_PARAMS/KAMIKAZE_*_PARAMS tablolarıyla
aynı dosyada: bir manevranın açtığı parametrenin baseline'da karşılığı yoksa
manevra bittiğinde o değer araçta kalır. Dosyanın sonundaki assert bunu
import anında yakalar.
"""
from enum import Enum

from src.MapWidget import CRUISE_THR_MAX, CRUISE_ROLL_LIMIT
from src.RoutePreplanner import CRUISE_SPEED_MS

# TKOFF_THR_MAX_T: kalkış koşusunun başındaki tam gaz süresi (ArduPlane 10 s'de
# sınırlıyor). Kalkış fazı TKOFF_ALT'a kadar sürer, sonra CRUISE_THR_MAX devralır.
TAKEOFF_FULL_THROTTLE_TIME: float = 10.0

# -------------------------------- HSS --------------------------------
# HER ZAMAN yürürlükte; kamikaze bunlardan etkilenmez, sınırların içine
# sığacak şekilde planlanır (bkz. KAMIKAZE_APPROACH_ALT).

FENCE_TYPE_BITS: float = 5.0     # 1=AltMax, 4=Polygon
FENCE_ACTION_RTL: float = 1.0    # FENCE_RET_RALLY=1 ile: en yakın rally noktasına git
FENCE_ALT_MAX_M: float = 150.0   # Yarışma tavanı
FENCE_ALT_MIN_M: float = 30.0    # Yarışma tabanı tahmini (bit kapalı, yukarı bak)
FENCE_MARGIN_M: float = 5.0      # Çite yaklaşma marjı
FENCE_LOITER_RADIUS_M: float = 60.0

# HSS güvenlik ağı + rally + AUTO seyrüsefer. Manevralarla değişmediği için
# baseline'a dahil değil; bağlantı kurulduğunda bir kez yazılır.
HSS_SAFETY_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('FENCE_ACTION',     [(b'FENCE_ACTION', FENCE_ACTION_RTL)]),
    ('FENCE_TYPE',       [(b'FENCE_TYPE', FENCE_TYPE_BITS)]),
    # Bit0=0: pilot/GCS fence ihlalinden sonra mod değiştirebilir.
    ('FENCE_OPTIONS',    [(b'FENCE_OPTIONS', 0.0)]),
    ('FENCE_MARGIN',     [(b'FENCE_MARGIN', FENCE_MARGIN_M)]),
    ('FENCE_RET_RALLY',  [(b'FENCE_RET_RALLY', 1.0)]),
    ('FENCE_ALT_MAX',    [(b'FENCE_ALT_MAX', FENCE_ALT_MAX_M)]),
    ('FENCE_ALT_MIN',    [(b'FENCE_ALT_MIN', FENCE_ALT_MIN_M)]),
    ('RALLY_LIMIT_KM',   [(b'RALLY_LIMIT_KM', 0.0)]),
    ('RALLY_INCL_HOME',  [(b'RALLY_INCL_HOME', 1.0)]),
    ('MIS_RESTART',      [(b'MIS_RESTART', 0.0)]), #0(Resume),1(Restart)
    ('WP_LOITER_RAD',    [(b'WP_LOITER_RAD', FENCE_LOITER_RADIUS_M)]),
    ('WP_MAX_RADIUS',    [(b'WP_MAX_RADIUS', 0.0)]), #Maksimum Waypoint Yarıçapı
    ('WP_RADIUS',        [(b'WP_RADIUS', 30.0)]), #Waypoint Kabul Yarıçapı
    ('TKOFF_THR_MAX',    [(b'TKOFF_THR_MAX', 100.0)]),
    ('TKOFF_THR_MAX_T',  [(b'TKOFF_THR_MAX_T', TAKEOFF_FULL_THROTTLE_TIME)]),
]


# --- Cruise / baseline zarfı -----------------------------------------------
# "Araç normal" durumunun TEK tanımı BASELINE_PARAMS (dosyanın altında). Zarfı
# açan her manevra onu apply_baseline_params() ile geri vermek zorunda; bitişe
# literal yazılırsa HSS'in belirlediği limit ilk koşudan sonra kaybolur.
CRUISE_PITCH_MIN: float = -25.0
# Araca yazılan seyir hava hızı; tek tanımı burasıdır (rota yükleme ve bağlantı
# kurma da bunu kullanır). HER ZAMAN CRUISE_SPEED_MS'ten küçük veya eşit olmalı:
# yavaş uçmak dönüş yarıçapını küçültür, plan tamponları güvenli tarafta kalır.
#
# Dalışa bu hızla giriliyor ve düşürmek dalış açısını ARTIRMAZ, azaltır: dalış
# oturmadığı için belirleyici olan burnun dönme hızı, o da hava hızıyla artan
# elevator otoritesine bağlı. 15 m/s'de açı 38'den 22'ye düştü (2026-08-14).
CRUISE_AIRSPEED_MS: float = 20.0
assert CRUISE_AIRSPEED_MS <= CRUISE_SPEED_MS, \
    "Seyir hızı rota planlayıcının varsaydığı hızı aşamaz, HSS tamponları yetersiz kalır"
# RoutePreplanner.compute_safe_route'taki turn_dist bunu varsayıyor; birlikte değişmeli.
CRUISE_NAVL1_PERIOD: float = 14.0

# --- Parametre yazma güvenilirliği -----------------------------------------
# param_set_send tek yönlü; paket düşerse araç eski değerde kalır. Kritik yazılar
# PARAM_VALUE ile teyit edilip gelmezse tekrarlanıyor (kurtarmadaki THR_MAX
# yazısının düşmesi motorsuz tırmanış demek). Süreler telsiz linkine göre; havada
# duran yazı sayısı da sınırlı, yoksa toplu gönderim linki kendi cevaplarına
# kapatıyor (28 parametrelik açılış yazımında cevaplar ~3 saniye sürüyordu).
PARAM_ACK_TIMEOUT: int = 1500      # ms, tek bir yazının cevap bekleme süresi
PARAM_MAX_ATTEMPTS: int = 4
PARAM_MAX_IN_FLIGHT: int = 5       # aynı anda cevabı beklenen yazı sayısı
PARAM_PUMP_INTERVAL: int = 200     # ms, kuyruğun ne sıklıkta işlendiği

# --- Kamikaze run ----------------------------------------------------------
# Koşunun tamamı GUIDED'da uçuluyor: otopilot kendi seyrüsefer yaptığı için
# heartbeat MAV_MODE_FLAG_AUTO_ENABLED bildiriyor ve koşu otonom sayılıyor.
#
# Yaklaşma irtifası iki taraftan sıkışık. Alttan: dip toparlanması tek başına
# (V^2/(g*(n-1)))*(1-cos(DIVE_ANGLE)) ≈ 30 m yiyor. Üstten: irtifa çiti koşu için
# askıya ALINMIYOR ve hem hizalanma hem kurtarma bu irtifada seviyeleniyor, yani
# FENCE_ALT_MAX'ın altında çit marjı + TECS'in tırmanışı durdurma payı kalmalı.
KAMIKAZE_ALT_FENCE_HEADROOM: float = 20.0
KAMIKAZE_APPROACH_ALT: float = min(100.0, FENCE_ALT_MAX_M - FENCE_MARGIN_M - KAMIKAZE_ALT_FENCE_HEADROOM)
# Dalışın APPROACH_ALT'ın ne kadar altından başlayabileceği. Daha alçaktan
# dalmak hem irtifa hem açıya dönme süresi bırakmaz. Bunun altında hizalanma
# devam eder: hedef her KAMIKAZE_TARGET_REFRESH'te yeniden hesaplandığı için
# uçak tırmanmaya devam ederken tur atar.
KAMIKAZE_APPROACH_ALT_TOLERANCE: float = 15.0
# Dalışın QR noktasına en yakın başlayabileceği mesafe. Gerçek başlangıç bununla
# "QR'ın burnun tam önüne düştüğü mesafe"nin uzak olanı: DIVE_ANGLE'lık alçalmada
# burun altitude/tan(DIVE_ANGLE) ileriyi gösterir, yani 45 derecede dalış uçak ne
# kadar yüksekse o kadar uzaktan başlamalı, yoksa kamera QR'a hiç bakmaz.
KAMIKAZE_DIVE_START_DISTANCE: float = 150.0
# GUIDED'a verilen hedef, uçağın üzerinde geldiği hattın QR noktasından bu kadar
# ilerisine konur. ModeGuided her zaman update_loiter() ile seyrediyor (GUIDED'da
# düz waypoint seyrüseferi hiç kullanılmıyor). L1'in çember yakalama terimi loiter
# çemberinin ~100 m dışında negatife dönüp uçağı çemberden uzağa yatırıyor; hedefi
# çok ileriye koymak terimi pozitif tutuyor ve L1 QR noktasının üstünden düz hat
# uçuyor.
KAMIKAZE_AIM_OVERSHOOT: float = 500.0
# Dalış açısı, dip toparlanmada inilebilecek en alçak irtifa ve koşunun bitmesi
# için kurtarma tırmanışının ulaşması gereken irtifa.
KAMIKAZE_DIVE_ANGLE: float = 45.0
KAMIKAZE_MIN_ALT: float = 50.0
KAMIKAZE_RECOVER_ALT: float = 90.0
# Dalış, MIN_ALT dibe vurulan yer olacak kadar erken kesiliyor: kurtarma
# komutundan sonra uçağın o anki alçalma hızıyla ne kadar daha battığı. Dairesel
# toparlanma yayı değil sadece tepki gecikmesi -- pitch tabanı seviyeye çekilince
# 22 m/s'lik dalış yalnızca ~3 m battı. Ölçüm (2026-08-14): tetik 79.6 m, alçalma
# 14.9 m/s, örnek yaşı 0.44 s, dip 67.0 m -> yaş düşülünce 0.41 s. Koşunun
# raporladığı dip irtifaya göre ayarlayın.
KAMIKAZE_PULLOUT_TIME: float = 0.40
# Pull-out tahmininde kullanılan alçalma hızı, bu kadar saniyelik pencerenin
# TEPESİ. Bkz. MainWindow.__recent_peak_sink.
KAMIKAZE_PULLOUT_PEAK_WINDOW: float = 1.5
# Kurtarmadaki pitch tabanı (derece). Dalışın -45'i burada kalırsa TECS burnu
# aşağıda tutup gereksiz irtifa yer; seviyeye çekmek MIN_ALT'ı tutturan şey.
# TECS'in hız kaybına verebileceği tek cevabı (burnu indirip hız toplamak)
# kapattığı için kendi başına stall riski; 50 metrede burun aşağı yetkisi
# vermemek operatör kararıdır (2026-08-21). Hava hızı düşüyorsa önce buraya bakın.
KAMIKAZE_RECOVER_PITCH_MIN: float = 0.0
# Alçalma hızının ölçüldüğü zaman penceresi (s) -- örnek SAYISI değil:
# GLOBAL_POSITION_INT (~2 Hz) döngüden (TICK_INTERVAL) çok yavaş geldiği için
# örnekler zaman damgalı ve yalnızca farklı irtifalar kaydediliyor.
KAMIKAZE_SINK_WINDOW: float = 0.6
KAMIKAZE_TICK_INTERVAL: int = 100
# Hizalanma, dalış ve kurtarma tırmanışındaki motor gücü (%). Kurtarma gazı
# bilerek CRUISE_THR_MAX'ın TA KENDİSİ: 90 iken girişte pervane tek adımda
# 0'dan %90'a çıkıp uçağı sola yuvarlıyor, çıkışta ise koşu bitince 60'a düşüp
# alçak irtifadaki tırmanışın ortasında gücü kesiyordu.
KAMIKAZE_APPROACH_THR_MAX: float = 60.0
KAMIKAZE_DIVE_THR_MAX: float = 0.0
KAMIKAZE_RECOVER_THR_MAX: float = CRUISE_THR_MAX
# Hizalanmadaki yatış limiti (derece). Hedef kerterizine oturması gerektiği için
# baseline'dan geniş; koşu bitince CRUISE_ROLL_LIMIT'e bırakılıyor.
KAMIKAZE_APPROACH_ROLL_LIMIT: float = 55.0
# Kurtarmadaki yatış limiti (derece). Dalışınki HSS bölgesinden kaçmaya yetmiyor,
# hizalanmanınki ise dip toparlanma için fazla gevşek (g çekerken sert yatmak yük
# faktörünü bindiriyor). 25'te stall hızı katkısı 1.05x, 35'te 1.10x'ti; uçağın
# tüm uçuştaki en yavaş anı burası olduğu için o fark doğrudan marj.
KAMIKAZE_RECOVER_ROLL_LIMIT: float = 25.0
# Dalıştaki yatış limiti. Hedef yaklaşırken L1 kerteriz konusunda çok sinirli
# oluyor ve kanat düşmesi kamerayı QR'dan kaçırıyor.
KAMIKAZE_DIVE_ROLL_LIMIT: float = 15.0
# GUIDED TECS ile uçuyor ve TECS kendi zarfı dışında gördüğü eğim talebini yok
# sayıyor. Üç limit birden açılmazsa dalış sığ ve salınımlı çıkıyor:
#  - TECS_SINK_MAX alçalma talebini, dolayısıyla açıyı asin(SINK_MAX/V) ile sınırlar.
#  - ARSPD_FBW_MAX aşılınca TECS hız yakmak için burnu kaldırır (motorsuz 45
#    derece dalış ~35 m/s'ye çıkıyor).
#  - TECS_SPDWEIGHT pitch'i irtifa ve hız arasında paylaştırır; gaz 0 iken TECS'in
#    hızı kontrol edecek başka yolu olmadığı için kendi alçalma talebiyle
#    kavga edip dalışı salındırıyor. 0 pitch'i tamamen irtifaya verir.
# CRUISE_ değerleri ArduPlane varsayılanları, koşu bitince geri yazılıyor.
# SINK_MAX bilerek uçağın uçabileceğinden yüksek, yani bağlayıcı kısıt değil:
# alçalmayı gerçekte pitch tabanı ve sürükleme sınırlıyor. Dalış sınırlanacaksa
# bu değil pitch tabanı değiştirilmeli.
KAMIKAZE_TECS_SINK_MAX: float = 40.0
KAMIKAZE_TECS_CLMB_MAX: float = 15.0
KAMIKAZE_ARSPD_FBW_MAX: float = 38.0
KAMIKAZE_TECS_SPDWEIGHT: float = 0.0
CRUISE_TECS_SINK_MAX: float = 5.0
CRUISE_TECS_CLMB_MAX: float = 5.0
CRUISE_ARSPD_FBW_MAX: float = 22.0
CRUISE_TECS_SPDWEIGHT: float = 1.0
# ALT_SLOPE_MIN (eski firmware: GLIDE_SLOPE_MIN) 0 iken irtifa eğimi tamamen
# kapanır: ArduPlane irtifa değişimini mesafeye yaymayı bırakıp hemen ister,
# dalışı şekillendiren tek şey TECS_SINK_MAX kalır. Hedefin 500 m ileride olması
# dalışı bu sayede sığlaştırmıyor. Eğim açıkken talep hedefe ~10 derecelik rampa
# çiziyor, dalışın kazandığı her metre uçağı kendi hedefinin ALTINA düşürüyor ve
# TECS geri itiyordu.
KAMIKAZE_GLIDE_SLOPE_MIN: float = 0.0
CRUISE_GLIDE_SLOPE_MIN: float = 15.0
# Altında telemetrinin alçalma hızını beslemesine güvenilmeyen yer hızı, ve
# hesaplanan değerin bir parametre yazısına değmesi için gereken değişim.
KAMIKAZE_MIN_VALID_SPEED: float = 5.0
# 0.5 iken talep ölçüm gürültüsünü kovalıyordu: açı yarım saniyede bir ±3 derece
# oynadığı için talep 24-31 m/s arasında gidip her seferinde bir yazı üretiyordu.
# TECS'in 3 s'lik zaman sabiti bu salınımı zaten geçirmiyor.
KAMIKAZE_SINK_STEP: float = 2.0
# TECS_SINK_MAX bir tavan, talep değil: TECS geriden gelip istenenin birkaç
# derece sığında oturuyor, o yüzden uçulan açı ölçülüp eksik kalan kadar fazlası
# isteniyor. En kötü duruma, yani en kısa dalışa göre boyutlandı: 50 m'lik irtifa
# bütçesi ~2.5 s sürüyor, TECS_TIME_CONST'tan KISA, yani uçulan şey oturmuş açı
# değil dönme geçici rejimi (ölçülen açının 45 yerine 38 çıkma sebebi).
#
# Trim [0, TRIM_MAX] ile kırpılı, yani yalnızca DIVE_ANGLE'dan FAZLASINI
# isteyebilir; kazancı abartmak sert giriş demek, kaçak değil. TRIM_MAX,
# DIVE_PITCH_MARGIN'e eşit kalmalı (aşağıda assert): trim'in isteyebileceği en dik
# açı pitch tabanını geçerse TECS yalnızca doyuma girer.
KAMIKAZE_DIVE_TRIM_MAX: float = 15.0
KAMIKAZE_DIVE_TRIM_GAIN: float = 1.2
# Yeni bir alçalma hızına oturma süresini TECS_TIME_CONST belirliyor, dalış ise
# birkaç saniye sürüyor: 5 s'lik varsayılanda dalışın çoğu açıya dönmekle geçer.
# 3, belgelenmiş aralığın tabanı.
KAMIKAZE_TECS_TIME_CONST: float = 3.0
CRUISE_TECS_TIME_CONST: float = 5.0
# TECS_VERT_ACC: hata düzeltilirken kullanılabilecek azami düşey ivme.
# Varsayılan 7 m/s²; talep 5'ten 40'a sıçradığında burnun dönüşü bununla
# sınırlanıyor ve ölçümde dalışın ilk 1.4 saniyesi (%24) 1.5 m/s'nin altında
# geçti. 10, belgelenmiş üst sınır; kısa dalışta her yarım saniye açıya gidiyor.
KAMIKAZE_TECS_VERT_ACC: float = 10.0
CRUISE_TECS_VERT_ACC: float = 7.0
# Dalışın, QR burna gelmeden bu kadar önce başlaması. Seviyeden DIVE_ANGLE'a
# dönmek bir iki saniye sürüyor ve o sırada uçak QR hattının üstünde kalıyor;
# bu pay dönmeyi QR burna gelmeden önce harcatıyor.
KAMIKAZE_DIVE_ROTATION_LEAD: float = 40.0
# Kamikaze video kaydı dalıştan bu kadar saniye ÖNCE başlar, kurtarmaya geçilince
# kapanır. Dalış tetiği mesafeyle verildiği için tetik mesafesi bir saniyelik yol
# (yer hızı x bu değer) kadar dışarı alınıyor. Böylece kayıt dalış öncesi düz
# uçuşu da içerir: hem başlangıç anını belgeler hem burnun dönüşünü baştan yakalar.
KAMIKAZE_VIDEO_LEAD_TIME: float = 1.0
# Pitch tabanının dalış açısının ne kadar altında olacağı. Taban tam DIVE_ANGLE'a
# sabitlenirse dalışa girmek için gereken pay kalmaz -- burnun bir an yol açısını
# geçmesi gerekiyor.
KAMIKAZE_DIVE_PITCH_MARGIN: float = 15.0
# Trim, uçağın uçmasına izin verilenden daha dik bir yol açısı isteyemez.
assert KAMIKAZE_DIVE_TRIM_MAX <= KAMIKAZE_DIVE_PITCH_MARGIN, \
    "Dalış trim'i pitch tabanının izin verdiğinden dik açı istiyor (TECS doyuma girer)"
# Guided hedefiyle gönderilen loiter yarıçapı. KAMIKAZE_DIVE_START_DISTANCE'in
# belirgin altında kalmalı, yoksa uçak dalış tetiklenmeden hedef çevresindeki
# loiter çemberine dönmeye başlar ve dalışa yatık girer.
KAMIKAZE_LOITER_RADIUS: float = 50.0
# Hizalanma sırasında guided hedefinin tekrar gönderilme sıklığı; sadece düşen
# komut için. Dalış sırasında ASLA kullanılmamalı: her DO_REPOSITION ArduPlane'e
# glide slope'u uçağın o anki konumundan yeniden kurduruyor, yani irtifa talebi
# uçağın bulunduğu yere geri sıçrıyor ve düzgün alçalma testereye dönüyor.
KAMIKAZE_TARGET_REFRESH: int = 2000
# Kurtarma tırmanışının hedefi uçağın bu kadar ilerisine konur ki hedefe geri
# dönmek yerine kanatlar düz çeksin. Gerekçesi KAMIKAZE_AIM_OVERSHOOT ile aynı.
KAMIKAZE_RECOVER_LEAD: float = 500.0
# Araca bu (göreli) irtifanın altında bir hedef asla verilmez.
KAMIKAZE_MIN_AIM_ALT: float = 5.0
# Dalış hedef kerterizinden bu kadar sapmayla başlıyorsa operatörü uyar: kameranın
# QR'a bakması için burnun hedefe dönük olması gerekiyor.
KAMIKAZE_MAX_DIVE_HEADING_ERROR: float = 45.0
# Bir koşunun tavan süresi (s), düğmeden bırakmaya. Hizalanmanın kendiliğinden
# biteceği yok -- irtifa yetmezse sonsuza kadar tur atar -- ve koşu sürerken HSS
# katmanı donuk, yani sınırsız koşu sınırsız kör nokta demek. Süre dolunca koşu
# iptal edilip araç baseline'a döndürülür.
KAMIKAZE_MAX_RUN_TIME: float = 120.0
# Düz ileri kurtarma noktası HSS bölgesine düştüğünde sırayla denenen kerteriz
# sapmaları (derece) ve mesafe oranları.
KAMIKAZE_RECOVER_HEADING_OFFSETS: tuple = (0.0, 25.0, -25.0, 50.0, -50.0)
KAMIKAZE_RECOVER_LEAD_FRACTIONS: tuple = (1.0, 0.6, 0.35)

# --- BASELINE: aracın "normal" durumunun tek tanımı ------------------------
# Bir manevranın dokunduğu HER parametre burada olmak zorunda; olmayan, manevra
# bittikten sonra araçta manevra değeriyle kalır.
#
# (kanonik_isim, [(mavlink_param, değer), ...]) — ikinci listedekiler birbirinin
# alias'ı (ArduPlane 4.1+ eski centidegree isimlerini değiştirdi), ikisi de
# yazılıyor ve HERHANGİ BİRİNDEN gelen PARAM_VALUE teyit sayılıyor.
BASELINE_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', CRUISE_THR_MAX)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', CRUISE_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', CRUISE_ROLL_LIMIT * 100.0)]),
    ('PTCH_LIM_MIN_DEG', [(b'PTCH_LIM_MIN_DEG', CRUISE_PITCH_MIN),
                          (b'LIM_PITCH_MIN', CRUISE_PITCH_MIN * 100.0)]),
    # 0, pitch tabanını LIM_PITCH_MIN'e geri bırakır.
    ('TECS_PITCH_MIN',   [(b'TECS_PITCH_MIN', 0.0)]),
    ('TECS_SINK_MAX',    [(b'TECS_SINK_MAX', CRUISE_TECS_SINK_MAX)]),
    ('TECS_CLMB_MAX',    [(b'TECS_CLMB_MAX', CRUISE_TECS_CLMB_MAX)]),
    # ArduPlane 4.4 ARSPD_FBW_MIN/MAX'i AIRSPEED_MIN/MAX yaptı; 4.4+ üzerinde
    # yalnızca eski ismi yazmak sessizce başarısız oluyor ve dalış ~35 derecede
    # kalıyordu.
    ('ARSPD_FBW_MAX',    [(b'AIRSPEED_MAX', CRUISE_ARSPD_FBW_MAX),
                          (b'ARSPD_FBW_MAX', CRUISE_ARSPD_FBW_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', CRUISE_TECS_SPDWEIGHT)]),
    ('TECS_TIME_CONST',  [(b'TECS_TIME_CONST', CRUISE_TECS_TIME_CONST)]),
    ('TECS_VERT_ACC',    [(b'TECS_VERT_ACC', CRUISE_TECS_VERT_ACC)]),
    # Yeni ArduPlane GLIDE_SLOPE_MIN'i ALT_SLOPE_MIN yaptı; aynı anlam, aynı birim.
    ('ALT_SLOPE_MIN',    [(b'ALT_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN),
                          (b'GLIDE_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN)]),
    # ArduPlane 4.4+ TRIM_ARSPD_CM'i (cm/s) AIRSPEED_CRUISE (m/s) yaptı.
    ('AIRSPEED_CRUISE',  [(b'AIRSPEED_CRUISE', CRUISE_AIRSPEED_MS),
                          (b'TRIM_ARSPD_CM', CRUISE_AIRSPEED_MS * 100.0)]),
    ('NAVL1_PERIOD',     [(b'NAVL1_PERIOD', CRUISE_NAVL1_PERIOD)]),
]



# --- Kamikaze fazlarının parametreleri -------------------------------------
# Buradaki HER kanonik isim BASELINE_PARAMS'ta da bulunmak zorunda (aşağıdaki
# assert kontrol ediyor). Dalış sırasında TECS_SINK_MAX ayrıca ve sürekli
# yazılıyor (MainWindow.__update_dive_sink_rate) -- o bir denetim döngüsü, faz
# ön ayarı değil, o yüzden bu tablolarda yok.

# Burun tabanı: dalışa girmek için yol açısını bir an geçmek gerekiyor.
KAMIKAZE_DIVE_PITCH_FLOOR: float = -(KAMIKAZE_DIVE_ANGLE + KAMIKAZE_DIVE_PITCH_MARGIN)

# Hedefe hizalanma fazı.
KAMIKAZE_APPROACH_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    # Süren bir reposition ya da önceki faz THR_MAX'ı başka yerde bırakmış
    # olabilir, o yüzden açıkça yazılıyor.
    ('THR_MAX',          [(b'THR_MAX', KAMIKAZE_APPROACH_THR_MAX)]),
    ('PTCH_LIM_MIN_DEG', [(b'PTCH_LIM_MIN_DEG', KAMIKAZE_DIVE_PITCH_FLOOR),
                          (b'LIM_PITCH_MIN', KAMIKAZE_DIVE_PITCH_FLOOR * 100.0)]),
    # TECS kendi pitch tabanını LIM_PITCH_MIN'in üstüne koyuyor ve dar olan
    # kazanıyor; ikisi birden açılmazsa GUIDED burnu seyir limitinin altına
    # hiç indirmiyor.
    ('TECS_PITCH_MIN',   [(b'TECS_PITCH_MIN', KAMIKAZE_DIVE_PITCH_FLOOR)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', KAMIKAZE_APPROACH_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', KAMIKAZE_APPROACH_ROLL_LIMIT * 100.0)]),
    ('TECS_CLMB_MAX',    [(b'TECS_CLMB_MAX', KAMIKAZE_TECS_CLMB_MAX)]),
    # Bu yazı düştüğünde TECS hızı seyir limitinde tutuyor ve dalış hiç gelişmiyor.
    ('ARSPD_FBW_MAX',    [(b'AIRSPEED_MAX', KAMIKAZE_ARSPD_FBW_MAX),
                          (b'ARSPD_FBW_MAX', KAMIKAZE_ARSPD_FBW_MAX)]),
    ('ALT_SLOPE_MIN',    [(b'ALT_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN),
                          (b'GLIDE_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN)]),
]

# Dalış. Motor kapalı, kanatlar sabitlenmiş, pitch tamamen irtifa talebinde:
# SPDWEIGHT 0 olmazsa TECS alçalma talebiyle hava hızını birbirine karşı oynatıp
# dalışı salındırıyor.
KAMIKAZE_DIVE_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', KAMIKAZE_DIVE_THR_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', KAMIKAZE_TECS_SPDWEIGHT)]),
    ('TECS_TIME_CONST',  [(b'TECS_TIME_CONST', KAMIKAZE_TECS_TIME_CONST)]),
    ('TECS_VERT_ACC',    [(b'TECS_VERT_ACC', KAMIKAZE_TECS_VERT_ACC)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', KAMIKAZE_DIVE_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', KAMIKAZE_DIVE_ROLL_LIMIT * 100.0)]),
]

# Kurtarma tırmanışı, seyir TECS'iyle uçuluyor. Dalışın açtığı zarf
# (ALT_SLOPE_MIN=0, CLMB_MAX=15, VERT_ACC=10, TIME_CONST=3) burada bırakılırsa
# "APPROACH_ALT'a olabildiğince sert çek" demek oluyor ve bu uçağın en yavaş
# anında, motorsuz bir dalıştan hemen sonra isteniyor; 2026-08-21 uçuşunda
# çıkışta 40-50 m'de sola yatıp takla atmasının en olası açıklaması bu.
# Özellikle ALT_SLOPE_MIN'in geri verilmesi belirleyici: irtifa farkı mesafeye
# yayıldığı için tırmanış basamak değil yumuşak rampa oluyor.
KAMIKAZE_RECOVER_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', KAMIKAZE_RECOVER_THR_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', CRUISE_TECS_SPDWEIGHT)]),
    # Tırmanışı şekillendiren dört ayar.
    ('ALT_SLOPE_MIN',    [(b'ALT_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN),
                          (b'GLIDE_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN)]),
    ('TECS_CLMB_MAX',    [(b'TECS_CLMB_MAX', CRUISE_TECS_CLMB_MAX)]),
    ('TECS_VERT_ACC',    [(b'TECS_VERT_ACC', CRUISE_TECS_VERT_ACC)]),
    ('TECS_TIME_CONST',  [(b'TECS_TIME_CONST', CRUISE_TECS_TIME_CONST)]),
    # Dalışta __update_dive_sink_rate bunu 30-40'a kadar yazıyor; açık bırakmak
    # TECS'e kurtarmada gereksiz bir alçalma serbestliği tanır.
    ('TECS_SINK_MAX',    [(b'TECS_SINK_MAX', CRUISE_TECS_SINK_MAX)]),
    ('PTCH_LIM_MIN_DEG', [(b'PTCH_LIM_MIN_DEG', KAMIKAZE_RECOVER_PITCH_MIN),
                          (b'LIM_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN * 100.0)]),
    ('TECS_PITCH_MIN',   [(b'TECS_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', KAMIKAZE_RECOVER_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', KAMIKAZE_RECOVER_ROLL_LIMIT * 100.0)]),
]

# Bir manevranın açtığı her parametrenin baseline'da karşılığı olmak zorunda.
_BASELINE_KEYS = {k for k, _ in BASELINE_PARAMS}
for _table, _name in ((KAMIKAZE_APPROACH_PARAMS, 'APPROACH'),
                      (KAMIKAZE_DIVE_PARAMS, 'DIVE'),
                      (KAMIKAZE_RECOVER_PARAMS, 'RECOVER')):
    _missing = [k for k, _ in _table if k not in _BASELINE_KEYS]
    assert not _missing, \
        "KAMIKAZE_%s_PARAMS içinde BASELINE_PARAMS'ta karşılığı olmayan " \
        "parametre var, koşudan sonra araçta kalır: %s" % (_name, _missing)

class ParamOwner(Enum):
    """Araç durumunun o anki sahibi. Bkz. MainWindow.__set_param."""
    BASELINE = 0
    KAMIKAZE = 1
