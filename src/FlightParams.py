"""
Uçuş zarfı sabitleri ve parametre tabloları.

Sabitler, onları araca yazan BASELINE_PARAMS/KAMIKAZE_*_PARAMS tablolarıyla
aynı dosyada tutuluyor: bir manevranın açtığı parametrenin baseline'da
karşılığı yoksa, manevra bittiğinde o değer araçta kalır. İkisi bir arada
olduğu için bu eksiklik hem gözle görülür hem de dosyanın sonundaki assert
ile import anında yakalanır.

Her sabitin üstündeki yorum değerin neden o olduğunu anlatır; uçuş
ölçümüne dayananlarda ölçüm tarihiyle birlikte yazılıdır.
"""
from enum import Enum

from src.MapWidget import CRUISE_THR_MAX, CRUISE_ROLL_LIMIT
from src.RoutePreplanner import CRUISE_SPEED_MS

# Seconds of forced full power at the start of the takeoff run (TKOFF_THR_MAX_T,
# which ArduPlane caps at 10). The takeoff stage itself keeps running until
# TKOFF_ALT is reached; only after it ends does CRUISE_THR_MAX take over.
TAKEOFF_FULL_THROTTLE_TIME: float = 10.0

# -------------------------------- HSS --------------------------------
# Bunlar HSS ile ilgili parametreler ve HER ZAMAN yürürlükte
# Kamikaze yapıldığında bunlardan etkilenmez,burada yazan sınırların içine sığacak şekilde planlanır (bkz. KAMIKAZE_APPROACH_ALT).


# FENCE_TYPE bitleri: 1=AltMax, 4=Polygon

FENCE_TYPE_BITS: float = 5.0

# FENCE_ACTION=1 + FENCE_RET_RALLY=1 ("en yakın rally noktasına git") 

FENCE_ACTION_RTL: float = 1.0
FENCE_ALT_MAX_M: float = 150.0   # Yarışma tavanı
FENCE_ALT_MIN_M: float = 30.0    # Yarışma tabanı tahmini (bit kapalı, yukarı bak)
FENCE_MARGIN_M: float = 5.0      # Çite yaklaşma marjı
FENCE_LOITER_RADIUS_M: float = 60.0

# HSS güvenlik ağı + rally + AUTO seyrüsefer parametreleri.Bunlar manevralarla
# değişmediği için baseline'a dahil değil; bağlantı kurulduğunda bir kez yazılır.
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
# "Araç normal, HSS'in planladığı hâlde" durumunun TEK tanımı BASELINE_PARAMS
# tablosu (bu dosyanın altında). Zarfı açan her manevra (kamikaze, reposition)
# onu apply_baseline_params() ile geri vermek zorunda; kendi literal'ini yazmak
# yasak. Manevra bitişine literal yazılırsa HSS'in belirlediği limit ilk
# koşudan sonra sessizce kaybolur.
CRUISE_PITCH_MIN: float = -25.0
# Araca yazılan seyir hava hızı. Tek tanımı burasıdır: rota yükleme ve bağlantı
# kurma yolları da bu değeri kullanmak zorunda, yoksa araç plandan farklı bir
# hızda uçar.
#
# Rota planlayıcının varsaydığı hızdan (RoutePreplanner.CRUISE_SPEED_MS = 20)
# BİLEREK düşük. Planlayıcı tamponlarını 20 m/s hava hızı + 8 m/s rüzgâr = 28
# m/s yer hızı üzerinden boyutluyor; daha yavaş uçmak dönüş yarıçapını
# küçültüyor, yani planın tamponları fazladan güvenli tarafta kalıyor.
# Değişmez kural: bu değer HER ZAMAN CRUISE_SPEED_MS'ten küçük veya eşit olmalı.
# Büyütülürse plan gerçekten daha keskin dönen bir uçak varsayıyor demektir ve
# HSS tamponları yetersiz kalır.
#
# Kamikaze açısından kritik: yaklaşma bu hızda uçuluyor, dalışa bu hızla
# giriliyor. Düşürmek dalış açısını ARTIRMAZ, azaltır. "Yavaş giriş = düşük yer
# hızı = aynı alçalma hızıyla daha dik açı" ilişkisi yalnızca oturmuş dalış için
# geçerli; dalış TECS_TIME_CONST'tan kısa sürdüğü için hiç oturmuyor.
# Belirleyici olan geçiş rejimi, yani burnun ne kadar hızlı aşağı dönebildiği --
# o da hava hızıyla artan elevator otoritesine bağlı. 15 m/s ölçümünde açı
# 38'den 22'ye düştü (2026-08-14). Hız düşürülecekse
# dalış süresi de uzatılmalı (irtifa bütçesi).
CRUISE_AIRSPEED_MS: float = 20.0
assert CRUISE_AIRSPEED_MS <= CRUISE_SPEED_MS, \
    "Seyir hızı rota planlayıcının varsaydığı hızı aşamaz, HSS tamponları yetersiz kalır"
# RoutePreplanner.compute_safe_route içindeki turn_dist hesabı bu değeri
# varsayıyor; ikisi birlikte değişmeli.
CRUISE_NAVL1_PERIOD: float = 14.0

# --- Parametre yazma güvenilirliği -----------------------------------------
# param_set_send tek yönlü; paket düşerse araç eski değerde kalır ve arayüz
# bunu göremez. Kritik yazılar PARAM_VALUE ile teyit edilip, gelmezse
# tekrarlanıyor. Kurtarmadaki THR_MAX yazısının düşmesi motorsuz tırmanış
# demek olduğu için bu isteğe bağlı bir iyileştirme değil.
PARAM_ACK_TIMEOUT: int = 700       # ms
PARAM_MAX_ATTEMPTS: int = 4

# --- Kamikaze run ----------------------------------------------------------
# The whole run is flown in GUIDED: the autopilot keeps navigating itself, so
# the heartbeat keeps reporting MAV_MODE_FLAG_AUTO_ENABLED and the run counts
# as autonomous. Altitude the run-in is flown at, and the distance to the
# target the dive is started from.
#
# The approach altitude is not free: the pull-out at the bottom of a DIVE_ANGLE
# dive costs (V^2/(g*(n-1)))*(1-cos(DIVE_ANGLE)) of altitude on its own, which
# is around 30 m at the ~35 m/s a zero-throttle 45 degree dive builds up. So
# APPROACH_ALT has to clear MIN_ALT by that pull-out plus however much dive is
# actually wanted in between.
#
# It is also capped from above by the altitude fence, which is NOT suspended for
# the run: the run-in and the recovery climb both level off at this altitude, so
# it has to sit below FENCE_ALT_MAX by the fence margin plus enough room for
# TECS to arrest the climb. Setting it at FENCE_ALT_MAX would put the whole
# run-in on the fence boundary, where a breach fights the 2 s DO_REPOSITION
# refresh for control of the vehicle.
KAMIKAZE_ALT_FENCE_HEADROOM: float = 20.0
KAMIKAZE_APPROACH_ALT: float = FENCE_ALT_MAX_M - FENCE_MARGIN_M - KAMIKAZE_ALT_FENCE_HEADROOM
# How far below APPROACH_ALT the dive is still allowed to start from. Diving
# from lower means less altitude to spend and less time to rotate into the
# angle, which is what makes a run started right after a previous one come out
# shallower. Below this the run-in simply carries on: the destination is
# recomputed from the current bearing to the QR point every
# KAMIKAZE_TARGET_REFRESH, so overflying the target turns the vehicle around
# for another pass while it keeps climbing. Set it large to dive from whatever
# altitude is available instead of going around.
KAMIKAZE_APPROACH_ALT_TOLERANCE: float = 15.0
# Closest the dive may ever start to the QR point. The dive actually starts at
# whichever is further out, this or the distance that puts the QR straight
# ahead on the nose: in a DIVE_ANGLE descent the nose points at a spot
# altitude/tan(DIVE_ANGLE) ahead, so at 45 degrees the dive has to start as far
# out as the vehicle is high or the camera never looks at the QR code.
KAMIKAZE_DIVE_START_DISTANCE: float = 150.0
# The destination handed to GUIDED is put this far past the QR point, along the
# line the vehicle is already running in on. ModeGuided always navigates with
# update_loiter() -- set_guided_WP() clears auto_state.crosstrack, so ArduPlane
# never uses straight waypoint navigation in GUIDED. L1's circle capture term
# is Kx*(distance-radius) - Kv*closing_speed, which goes negative roughly 100 m
# outside the loiter circle and banks the vehicle away from it. Aiming well
# beyond the target keeps that term positive, which is what makes L1 fall back
# to its capture law and fly a straight line through the QR point instead.
KAMIKAZE_AIM_OVERSHOOT: float = 500.0
# Dive angle, the lowest altitude the vehicle may reach at the bottom of the
# pull-out, and the altitude the recovery climb has to reach before the run is
# over.
KAMIKAZE_DIVE_ANGLE: float = 45.0
KAMIKAZE_MIN_ALT: float = 70.0
KAMIKAZE_RECOVER_ALT: float = 100.0
# The dive is broken off early enough that MIN_ALT is where the vehicle bottoms
# out, not where it starts pulling: how much further it sinks after the
# recovery is commanded, as a time at the current sink rate.
#
# This was a circular pull-out arc, V^2/(g*(n-1))*(1-cos(angle)), which at any
# sane load factor predicts 20-30 m. Measured in flight it is nothing like
# that: clamping the pitch floor to level at recovery makes the pull so tight
# that a 22 m/s dive only sank about 3 m more, so the arc term is dropped and
# what is left is the response delay. Tune it against the altitude the run
# reports bottoming out at when it ends -- below MIN_ALT means raise this,
# well above it means the dive is still being cut short.
#
# Ölçüm (2026-08-14): tetik 79.6 m'de, alçalma 14.9 m/s, örnek yaşı 0.44 s,
# dip 67.0 m. Toplam kayıp 12.6 m = 0.85 s; yaş telafisi düşülünce aracın
# gerçek tepki gecikmesi 0.41 s çıkıyor. Yaş artık ayrıca hesaba katıldığı
# için bu sabit SADECE o gecikmeyi temsil ediyor.
KAMIKAZE_PULLOUT_TIME: float = 0.40
# Pull-out tahmininde kullanılan alçalma hızı, bu kadar saniyelik pencerenin
# TEPESİ. Bkz. MainWindow.__recent_peak_sink.
KAMIKAZE_PULLOUT_PEAK_WINDOW: float = 1.5
# Pitch floor (deg) during the recovery. The dive opens the floor to -45, and
# leaving it there lets TECS keep the nose down through the pull-out and eat
# altitude it does not need to. Clamping it at level makes the recovery as
# tight as the airframe allows, which is what keeps MIN_ALT honest.
KAMIKAZE_RECOVER_PITCH_MIN: float = 0.0
# Time window (s) the sink rate is measured over. NOT a sample count: the loop
# ticks at KAMIKAZE_TICK_INTERVAL but GLOBAL_POSITION_INT arrives far slower
# (measured at ~2 Hz on a real link), so most ticks carry a repeat of the last
# altitude. Counting repeats and dividing by the tick interval invents a rate --
# it reported 0 between samples and a 17-27 m/s spike on the tick a new sample
# landed, and the pull-out decision was being made off those spikes. Samples are
# now timestamped and only distinct altitudes are recorded, so this window works
# at any telemetry rate.
KAMIKAZE_SINK_WINDOW: float = 0.6
KAMIKAZE_TICK_INTERVAL: int = 100
# Motor power (%) during the run-in, the dive and the recovery climb.
KAMIKAZE_APPROACH_THR_MAX: float = 60.0
KAMIKAZE_DIVE_THR_MAX: float = 0.0
KAMIKAZE_RECOVER_THR_MAX: float = 90.0
# Bank limit (deg) for the run-in. More than the planned-route baseline because
# the run-in has to line up on the target bearing; released back to
# CRUISE_ROLL_LIMIT at the end.
KAMIKAZE_APPROACH_ROLL_LIMIT: float = 55.0
# Bank limit (deg) for the recovery. The dive pins it at DIVE_ROLL_LIMIT, which
# is too tight to steer around an HSS zone, but the run-in value is too loose
# for the bottom of the pull-out: banking hard while already pulling g stacks
# the load factor. This is enough to fly the largest heading offset
# KAMIKAZE_RECOVER_HEADING_OFFSETS asks for and no more.
KAMIKAZE_RECOVER_ROLL_LIMIT: float = 35.0
# Bank limit (deg) while diving. L1 gets very twitchy about bearing as the
# target gets close, and a wing drop points the camera off the QR code, so the
# dive is flown with barely enough roll authority to hold the line.
KAMIKAZE_DIVE_ROLL_LIMIT: float = 15.0
# GUIDED flies on TECS, and TECS ignores a target slope it considers outside
# its own envelope. Three separate limits have to be opened or the dive comes
# out shallow and unsteady:
#  - TECS_SINK_MAX caps the demanded descent rate, which caps the dive angle at
#    asin(SINK_MAX/V): 20 m/s at 40 m/s airspeed is only 30 degrees.
#  - ARSPD_FBW_MAX makes TECS raise the nose to bleed speed once passed, and a
#    zero-throttle 45 degree dive builds up to roughly 35 m/s.
#  - TECS_SPDWEIGHT splits pitch between holding the altitude demand and
#    controlling airspeed. With the throttle at 0 TECS has no other way to
#    control speed, so it fights its own descent demand with the elevator and
#    the dive oscillates. 0 puts pitch fully on the altitude demand.
# The CRUISE_ values below are put back when the run ends and are the ArduPlane
# defaults, so adjust them if the airframe is tuned differently.
#
# SINK_MAX is deliberately set above anything the airframe will actually fly:
# the achievable angle is atan(SINK_MAX/groundspeed), so a 30 m/s ceiling caps
# the dive at 38 degrees once the groundspeed reaches 38 m/s -- which is exactly
# what was measured. What really limits the descent is the pitch floor
# (-(DIVE_ANGLE + DIVE_PITCH_MARGIN) = -60 deg) and drag, so this is left high
# enough not to be the binding constraint and the angle is shaped by the trim
# loop instead. Raise the pitch floor, not this, if the dive needs limiting.
KAMIKAZE_TECS_SINK_MAX: float = 40.0
KAMIKAZE_TECS_CLMB_MAX: float = 15.0
KAMIKAZE_ARSPD_FBW_MAX: float = 38.0
KAMIKAZE_TECS_SPDWEIGHT: float = 0.0
CRUISE_TECS_SINK_MAX: float = 5.0
CRUISE_TECS_CLMB_MAX: float = 5.0
CRUISE_ARSPD_FBW_MAX: float = 22.0
CRUISE_TECS_SPDWEIGHT: float = 1.0
# ALT_SLOPE_MIN (older firmware: GLIDE_SLOPE_MIN) at 0 turns off the altitude
# slope entirely: ArduPlane stops spreading an altitude change over the distance
# to the destination and demands it right away, leaving TECS_SINK_MAX as the
# only thing shaping the descent. That is what lets the destination sit 500 m
# past the target without dragging the dive shallow -- the angle comes from the
# sink rate, not from where the destination happens to be. 15 is the default.
#
# This write was silently failing until the rename was found. With the slope
# left active the demanded altitude followed a ~10 degree ramp to a destination
# 665 m away, so every metre the dive gained on that ramp put the vehicle BELOW
# its own altitude target and TECS pushed back -- which is the most likely
# reason the dive settled at 38 degrees while being asked for 24 m/s of sink
# and only delivering 15.
KAMIKAZE_GLIDE_SLOPE_MIN: float = 0.0
CRUISE_GLIDE_SLOPE_MIN: float = 15.0
# Ground speed below which telemetry is not trusted to slave the sink rate to,
# and how much the slaved value has to move before it is worth a parameter
# write.
KAMIKAZE_MIN_VALID_SPEED: float = 5.0
# 0.5 iken talep ölçüm gürültüsünü kovalıyordu: ölçülen açı yarım saniyede bir
# ±3 derece oynadığı için talep 24 ile 31 m/s arasında gidip geliyor ve her
# seferinde bir parametre yazısı üretiyordu. TECS'in zaman sabiti 3 s, yani bu
# salınımı zaten alçak geçiriyor -- tek etkisi boşuna telsiz trafiği.
KAMIKAZE_SINK_STEP: float = 2.0
# TECS_SINK_MAX is a ceiling, not a demand: TECS lags it by its own time
# constant and settles a few degrees shallow of whatever is asked for. So the
# angle actually being flown is measured and the ceiling is asked for the
# shortfall on top, which both pushes the nose over harder at the entry and
# trims the settled dive onto DIVE_ANGLE.
#
# These are sized against the WORST case, which is the shortest dive: the dive
# ends on altitude, not on distance, so the altitude budget
# (APPROACH_ALT - MIN_ALT) decides how long it lasts. At the current 55 m budget
# that is roughly 2.5 s -- LESS than TECS_TIME_CONST below, so the vehicle never
# reaches its settled angle and what gets flown is the rotation transient. That
# is what put the measured angle at 38 instead of 45. The trim therefore has to
# be aggressive enough to win inside one time constant.
#
# The trim is clamped to [0, TRIM_MAX], i.e. it can only ever ask for MORE than
# DIVE_ANGLE, never less: once the flown angle passes DIVE_ANGLE the term goes
# to zero and the demand falls back to exactly DIVE_ANGLE. So overshooting the
# gain costs a slightly abrupt entry, not a runaway.
#
# TRIM_MAX must stay equal to KAMIKAZE_DIVE_PITCH_MARGIN (asserted below): the
# steepest flight path the trim may demand is DIVE_ANGLE + TRIM_MAX, and the
# steepest the vehicle is allowed to fly is the pitch floor,
# -(DIVE_ANGLE + DIVE_PITCH_MARGIN). Demanding past the floor only saturates
# the elevator and winds TECS up against a limit it can never reach.
KAMIKAZE_DIVE_TRIM_MAX: float = 15.0
KAMIKAZE_DIVE_TRIM_GAIN: float = 1.2
# TECS_TIME_CONST is what sets how long the vehicle takes to settle onto a new
# descent rate, and a dive only lasts a few seconds -- at the 5 s default most
# of the dive is spent rotating into the angle rather than holding it. 3 is the
# bottom of the documented range.
KAMIKAZE_TECS_TIME_CONST: float = 3.0
CRUISE_TECS_TIME_CONST: float = 5.0
# TECS_VERT_ACC, yükseklik/hız hatasını düzeltirken kullanılabilecek azami
# düşey ivme. Varsayılan 7 m/s²; talep 5'ten 40'a sıçradığında burnun aşağı
# dönmesi bu ivmeyle sınırlanıyor ve ölçümde dalışın ilk 1.4 saniyesi (toplam
# sürenin %24'ü) alçalma hızı 1.5 m/s'nin altında geçti. 10, parametrenin
# belgelenmiş üst sınırı. Kısa bir dalışta kazanılan her yarım saniye
# doğrudan açıya gidiyor.
KAMIKAZE_TECS_VERT_ACC: float = 10.0
CRUISE_TECS_VERT_ACC: float = 7.0
# Extra distance the dive starts ahead of the nose-on point. Rotating from
# level to DIVE_ANGLE takes a second or two, and during it the vehicle is
# shallower than the line to the QR point, so the QR sits below the nose. This
# spends the rotation before the QR comes onto the nose instead of during it.
KAMIKAZE_DIVE_ROTATION_LEAD: float = 40.0
# Pitch floor headroom (deg) below the dive angle. Pinning the floor at exactly
# DIVE_ANGLE leaves nothing for establishing the dive: the nose has to go past
# the flight path angle for a moment to get there, and without that margin the
# vehicle only ever creeps up on the angle.
KAMIKAZE_DIVE_PITCH_MARGIN: float = 15.0
# Trim, uçağın uçmasına izin verilenden daha dik bir yol açısı isteyemez.
assert KAMIKAZE_DIVE_TRIM_MAX <= KAMIKAZE_DIVE_PITCH_MARGIN, \
    "Dalış trim'i pitch tabanının izin verdiğinden dik açı istiyor (TECS doyuma girer)"
# Loiter radius sent with the guided target. It has to stay well below
# KAMIKAZE_DIVE_START_DISTANCE, otherwise the vehicle starts turning onto its
# loiter circle around the target before the dive triggers and enters it while
# banked away.
KAMIKAZE_LOITER_RADIUS: float = 50.0
# How often the guided destination is repeated during the run-in, purely so a
# dropped command still gets through. It must never be used while diving: every
# DO_REPOSITION makes ArduPlane rebuild its glide slope from the vehicle's
# current position, which snaps the altitude demand back up to where the
# vehicle already is. Repeating it mid-dive turns a steady descent demand into
# a sawtooth and the vehicle porpoises down instead of tracking the slope.
KAMIKAZE_TARGET_REFRESH: int = 2000
# The recovery climb is flown towards a point this far ahead of the vehicle so
# it pulls up wings level instead of turning back towards the target. Same
# reasoning as KAMIKAZE_AIM_OVERSHOOT: far enough that L1 never starts the
# circle capture.
KAMIKAZE_RECOVER_LEAD: float = 500.0
# Never ask the vehicle to fly to a target below this (relative) altitude.
KAMIKAZE_MIN_AIM_ALT: float = 5.0
# Warn the operator if the dive starts this far off the target bearing: the
# camera needs the nose pointed at the QR code.
KAMIKAZE_MAX_DIVE_HEADING_ERROR: float = 45.0
# Hard ceiling (s) on a whole run, button press to release. The run-in has no
# natural end of its own -- if the vehicle is too low to dive it recomputes the
# destination and goes around, forever. While the run owns the vehicle the HSS
# layer is frozen (no fence uploads, no replanning), so an unbounded run means
# an unbounded blind spot. On expiry the run is cancelled and the vehicle is put
# back into its baseline state.
KAMIKAZE_MAX_RUN_TIME: float = 120.0
# Candidate heading offsets (deg) and lead-distance fractions tried, in order,
# when the straight-ahead recovery lead point falls inside an HSS zone.
KAMIKAZE_RECOVER_HEADING_OFFSETS: tuple = (0.0, 25.0, -25.0, 50.0, -50.0)
KAMIKAZE_RECOVER_LEAD_FRACTIONS: tuple = (1.0, 0.6, 0.35)

# --- BASELINE: aracın "normal" durumunun tek tanımı ------------------------
# Kamikaze veya reposition ne açtıysa buradan kapanır. Bir manevranın dokunduğu
# HER parametre burada olmak zorunda; olmayan bir parametre, manevra bittikten
# sonra araçta manevra değeriyle kalır.
#
# (kanonik_isim, [(mavlink_param, değer), ...]) — ikinci listedeki isimler
# birbirinin alias'ı; ArduPlane 4.1+ eski centidegree isimlerini yeniden
# adlandırdığı için ikisi de yazılıyor ve HERHANGİ BİRİNDEN gelen PARAM_VALUE
# teyit sayılıyor (firmware'de olmayan alias asla cevap vermez).
BASELINE_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', CRUISE_THR_MAX)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', CRUISE_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', CRUISE_ROLL_LIMIT * 100.0)]),
    ('PTCH_LIM_MIN_DEG', [(b'PTCH_LIM_MIN_DEG', CRUISE_PITCH_MIN),
                          (b'LIM_PITCH_MIN', CRUISE_PITCH_MIN * 100.0)]),
    # 0 hands the pitch floor back to LIM_PITCH_MIN.
    ('TECS_PITCH_MIN',   [(b'TECS_PITCH_MIN', 0.0)]),
    ('TECS_SINK_MAX',    [(b'TECS_SINK_MAX', CRUISE_TECS_SINK_MAX)]),
    ('TECS_CLMB_MAX',    [(b'TECS_CLMB_MAX', CRUISE_TECS_CLMB_MAX)]),
    # ArduPlane 4.4 renamed the airspeed limits: ARSPD_FBW_MIN/MAX became
    # AIRSPEED_MIN/MAX (same units, m/s), in the same rename as
    # TRIM_ARSPD_CM -> AIRSPEED_CRUISE. Writing only the old name on 4.4+ fails
    # silently, which is exactly what pinned the dive: the kamikaze run raises
    # this limit so the dive can build speed, and with the write lost TECS held
    # the nose up at the cruise limit and the dive stalled out at ~35 degrees.
    ('ARSPD_FBW_MAX',    [(b'AIRSPEED_MAX', CRUISE_ARSPD_FBW_MAX),
                          (b'ARSPD_FBW_MAX', CRUISE_ARSPD_FBW_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', CRUISE_TECS_SPDWEIGHT)]),
    ('TECS_TIME_CONST',  [(b'TECS_TIME_CONST', CRUISE_TECS_TIME_CONST)]),
    ('TECS_VERT_ACC',    [(b'TECS_VERT_ACC', CRUISE_TECS_VERT_ACC)]),
    # Newer ArduPlane renamed GLIDE_SLOPE_MIN to ALT_SLOPE_MIN (and
    # GLIDE_SLOPE_THR to ALT_SLOPE_MAXHGT). Same meaning, same units, same
    # default of 15 m. Confirmed against the vehicle's own parameter list.
    ('ALT_SLOPE_MIN',    [(b'ALT_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN),
                          (b'GLIDE_SLOPE_MIN', CRUISE_GLIDE_SLOPE_MIN)]),
    # ArduPlane 4.4+ renamed TRIM_ARSPD_CM (cm/s) to AIRSPEED_CRUISE (m/s).

    ('AIRSPEED_CRUISE',  [(b'AIRSPEED_CRUISE', CRUISE_AIRSPEED_MS),
                          (b'TRIM_ARSPD_CM', CRUISE_AIRSPEED_MS * 100.0)]),
    ('NAVL1_PERIOD',     [(b'NAVL1_PERIOD', CRUISE_NAVL1_PERIOD)]),
]



# --- Kamikaze fazlarının parametreleri -------------------------------------
# Koşunun üç fazı, BASELINE_PARAMS ile aynı biçimde. Buradaki HER kanonik isim
# BASELINE_PARAMS'ta da bulunmak zorunda, yoksa koşu bittiğinde o parametre
# manevra değeriyle araçta kalır; bunu aşağıdaki assert kontrol ediyor.
#
# Dalış sırasında TECS_SINK_MAX ayrıca ve sürekli yazılıyor
# (MainWindow.__update_dive_sink_rate) -- o bir denetim döngüsü, faz ön ayarı
# değil, o yüzden bu tablolarda yok.

# Burun tabanı: uçağın dalışa girebilmesi için yol açısını bir an geçmesi
# gerekiyor, o yüzden taban dalış açısının DIVE_PITCH_MARGIN kadar altında.
KAMIKAZE_DIVE_PITCH_FLOOR: float = -(KAMIKAZE_DIVE_ANGLE + KAMIKAZE_DIVE_PITCH_MARGIN)

# Hedefe hizalanma fazı. Eski (centidegree: LIM_*) ve yeni (degree: *_DEG)
# ArduPlane isimleri birlikte yazılıyor; 4.1+ bunları yeniden adlandırdı.
KAMIKAZE_APPROACH_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    # Hâlâ süren bir reposition ya da önceki faz THR_MAX'ı başka yerde bırakmış
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
    # Bu yazı düştüğünde TECS hızı seyir limitinde tutuyor ve dalış hiç
    # gelişmiyor -- isim değişikliği bulunana kadar sessizce olan buydu.
    ('ARSPD_FBW_MAX',    [(b'AIRSPEED_MAX', KAMIKAZE_ARSPD_FBW_MAX),
                          (b'ARSPD_FBW_MAX', KAMIKAZE_ARSPD_FBW_MAX)]),
    ('ALT_SLOPE_MIN',    [(b'ALT_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN),
                          (b'GLIDE_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN)]),
]

# Dalış. Motor kapalı, kanatlar sabitlenmiş ve pitch tamamen irtifa talebine
# bırakılmış: SPDWEIGHT 0 olmazsa TECS alçalma talebiyle hava hızını
# birbirine karşı oynatıp dalışı salındırıyor.
KAMIKAZE_DIVE_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', KAMIKAZE_DIVE_THR_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', KAMIKAZE_TECS_SPDWEIGHT)]),
    ('TECS_TIME_CONST',  [(b'TECS_TIME_CONST', KAMIKAZE_TECS_TIME_CONST)]),
    ('TECS_VERT_ACC',    [(b'TECS_VERT_ACC', KAMIKAZE_TECS_VERT_ACC)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', KAMIKAZE_DIVE_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', KAMIKAZE_DIVE_ROLL_LIMIT * 100.0)]),
]

# Kurtarma tırmanışı. Pitch tabanı seviyeye çekiliyor: dalışın -60'ı burada
# kalırsa TECS burnu aşağıda tutup gereksiz irtifa yiyor ve MIN_ALT tutmuyor.
# Yatış yetkisi kısmen geri veriliyor -- bkz. KAMIKAZE_RECOVER_ROLL_LIMIT.
KAMIKAZE_RECOVER_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('THR_MAX',          [(b'THR_MAX', KAMIKAZE_RECOVER_THR_MAX)]),
    ('TECS_SPDWEIGHT',   [(b'TECS_SPDWEIGHT', CRUISE_TECS_SPDWEIGHT)]),
    ('PTCH_LIM_MIN_DEG', [(b'PTCH_LIM_MIN_DEG', KAMIKAZE_RECOVER_PITCH_MIN),
                          (b'LIM_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN * 100.0)]),
    ('TECS_PITCH_MIN',   [(b'TECS_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN)]),
    ('ROLL_LIMIT_DEG',   [(b'ROLL_LIMIT_DEG', KAMIKAZE_RECOVER_ROLL_LIMIT),
                          (b'LIM_ROLL_CD', KAMIKAZE_RECOVER_ROLL_LIMIT * 100.0)]),
]

# Bir manevranın açtığı her parametrenin baseline'da bir karşılığı olmak
# zorunda; olmayan, koşu bittiğinde araçta manevra değeriyle kalır.
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
