import copy
import math
import os
import re
import time
from enum import Enum
from functools import partial

from src.CameraWidget import CameraServerProtocol
from src.CommonUtils import TrackableDataPacketTimer, main_and_sub_mode_to_px4_uav_mode, MSG_ID_2_TRACKABLE_DATA_TYPE, \
    PX4_UAV_Modes, KamikazeState, index_to_px4_uav_mode, SupportedLanguages, Ardupilot_UAV_Modes, HssSnapshot

os.environ['MAVLINK20'] = '1'

from PySide6.QtCore import QTimer, QModelIndex, qInfo, qWarning, QDateTime, qDebug, QThread, QObject, Signal, QLocale, \
    QTranslator, QCoreApplication, QRegularExpression, QRunnable, QThreadPool
from PySide6.QtGui import QAction, QDoubleValidator, Qt
from PySide6.QtPositioning import QGeoCoordinate
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import QMainWindow, QTableWidgetItem, QMenu, QApplication, QMessageBox, QStyle, QProxyStyle
from pymavlink.dialects.v20.all import MAVLink_gps_raw_int_message, MAVLink_attitude_message, \
    MAVLink_vfr_hud_message, MAVLink_battery_status_message, MAVLink_message, MAVLink_heartbeat_message, \
    MAVLink_global_position_int_message, MAVLink_system_time_message, MAV_CMD_DO_FENCE_ENABLE, \
    MAVLink_fence_status_message, \
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, MAV_FRAME_GLOBAL_INT, \
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAV_MISSION_TYPE_FENCE, \
    MAVLINK_MSG_ID_MISSION_ACK, MAVLINK_MSG_ID_MISSION_REQUEST, MAV_FRAME_GLOBAL, MAVLINK_MSG_ID_BAD_DATA, \
    MAVLINK_MSG_ID_COMMAND_ACK, MAV_CMD_DO_REPOSITION, MAV_RESULT_DENIED, PLANE_MODE_GUIDED, \
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, \
    MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_ITEM_INT, MAVLink_mission_item_int_message, \
    MAV_MISSION_ACCEPTED, MAVLINK_MSG_ID_MISSION_ITEM, MAVLink_mission_item_message, MAV_MODE_FLAG_SAFETY_ARMED, \
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_AUTOPILOT_INVALID, MAV_DATA_STREAM_ALL, \
    MAV_CMD_SET_MESSAGE_INTERVAL, MAV_MISSION_TYPE_MISSION, MAV_RESULT_TEMPORARILY_REJECTED, MAV_CMD_DO_SET_MODE, \
    MAV_MODE_FLAG_AUTO_ENABLED, MAV_AUTOPILOT_PX4, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, \
    MAV_RESULT_FAILED, MAV_RESULT_ACCEPTED, MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MISSION_CURRENT, MAV_TYPE_GCS, \
    MAVLINK_MSG_ID_PARAM_VALUE
from pymavlink.mavutil import mavfile, all_printable, mavtcp, mavudp, mavserial

from src.AddADSInterface import AddADSInterface
from src.CameraServerConnectionInterface import CameraServerConnectionInterface
from src.ColorSelectorInterface import ColorSelectorInterface, ColorOptions
from src.MapWidget import ZERO_GEO_COORDS, AdsData, SpecialCoordsData, CRUISE_THR_MAX, CRUISE_ROLL_LIMIT
from src.SetGeofenceInterface import SetGeofenceInterface
from src.FightingUAVConnectionInterface import FightingUAVConnectionInterface, ConnectionType
import src.ServerConnection as server_api
from src.ServerConnection import login_to_server, GpsSaati, send_telemetry, QrCoords, \
    get_kamikaze_coords, TelemetryData, TelemetryResponseData, get_ads, send_kamikaze, \
    SERVER_IS_UNREACHABLE_COUNTER, ServerAdsData
from src.ServerConnectionInterface import ServerConnectionInterface
from src.KeybindingConfigInterface import KeybindingConfigInterface
from src.input_types import InputMapping, KeybindingsEnum
from src.HSSPollingWorker import HSSPollingWorker
from src.RoutePreplanner import compute_safe_route, fence_radius_for_hss, point_hits_zone, segment_hits_zone, \
    CRUISE_SPEED_MS, RoutePlanResult
from ui_files_python.uav_interface import Ui_MainWindow

def to_degree(x: float) -> float:
    if x < 0:
        x = x + 2 * math.pi
    return x * (180 / math.pi)

def clamp(val: float, minv: float, maxv: float):
    return max(minv, min(maxv, val))

# Seconds of forced full power at the start of the takeoff run (TKOFF_THR_MAX_T,
# which ArduPlane caps at 10). The takeoff stage itself keeps running until
# TKOFF_ALT is reached; only after it ends does CRUISE_THR_MAX take over.
TAKEOFF_FULL_THROTTLE_TIME: float = 10.0

# --- HSS güvenlik ağı (geofence) -------------------------------------------
# Bunlar HSS katmanının malı ve HER ZAMAN yürürlükte: kamikaze koşusu bunları
# devralmaz, bunların İÇİNE sığacak şekilde planlanır (bkz. KAMIKAZE_APPROACH_ALT).
# Böylece güvenlik ağı en riskli fazda bile kapanmıyor.
#
# FENCE_TYPE bitleri: 1=AltMax, 2=Home merkezli daire (FENCE_RADIUS), 4=Polygon,
# 8=AltMin. Sunucudan gelen HSS bölgeleri exclusion CIRCLE olarak yükleniyor ve
# bunlar poly-fence yükleyicisine, yani bit2'ye (4) bağlı -- bit1 (2) DEĞİL.
# Bit1 hiç set edilmemiş FENCE_RADIUS'u (varsayılan 300 m) kullanan eski
# home-daire çiti; açık kalırsa uçak home'dan 300 m uzaklaşınca ihlal verir.
# Bu yüzden 7 değil 5.
# AltMin biti (8) bilerek kapalı: kamikaze dalışı FENCE_ALT_MIN'in üstünde
# bitiyor ama yerde/kalkışta çit etkinken anında ihlal üretirdi. Açılacaksa
# FENCE_AUTOENABLE ile birlikte değerlendirilmeli.
FENCE_TYPE_BITS: float = 5.0
# ArduPlane'de geçerli FENCE_ACTION değerleri 0 (sadece rapor), 1 (RTL),
# 6 (Guided), 7 (GuidedThrottlePass). Eskiden 4 yazılıyordu; 4 Copter'ın değeri,
# Plane'de tanınmadığı için çit fiilen hiçbir şey yapmıyordu. İstenen davranış
# ("en yakın rally noktasına git") FENCE_ACTION=1 + FENCE_RET_RALLY=1 ile olur.
FENCE_ACTION_RTL: float = 1.0
FENCE_ALT_MAX_M: float = 150.0   # Yarışma tavanı
FENCE_ALT_MIN_M: float = 30.0    # Yarışma tabanı tahmini (bit kapalı, yukarı bak)
FENCE_MARGIN_M: float = 5.0      # Çite yaklaşma marjı
FENCE_LOITER_RADIUS_M: float = 60.0

# --- Cruise / baseline zarfı -----------------------------------------------
# "Araç normal, HSS'in planladığı hâlde" durumunun TEK tanımı BASELINE_PARAMS
# tablosu (bu dosyanın altında). Zarfı açan her manevra (kamikaze, reposition)
# onu apply_baseline_params() ile geri vermek zorunda; kendi literal'ini yazmak
# yasak. Eskiden kamikaze bitişi roll limitini 55'e çekiyordu ve HSS'in 45'i
# ilk koşudan sonra sessizce kayboluyordu -- tablo bunu yapısal olarak engelliyor.
CRUISE_PITCH_MIN: float = -25.0
# Araca yazılan seyir hava hızı. Eskiden bunun tek tanımı yoktu: rota yüklemesi
# 15 m/s yazıp bir daha geri koymuyordu, bağlantı bloğu ise hiç yazmıyordu.
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
# giriliyor. Bir ara 15 m/s denendi, gerekçe "yavaş giriş = daha düşük yer hızı
# = aynı alçalma hızıyla daha dik açı" idi. Bu YANLIŞ çıktı: o ilişki oturmuş
# dalış için geçerli, ama dalış TECS_TIME_CONST'tan kısa olduğu için hiç
# oturmuyor. Belirleyici olan geçiş rejimi, yani burnun ne kadar hızlı aşağı
# dönebildiği -- ve o da hava hızıyla artan elevator otoritesine bağlı. Yavaş
# girmek dönüşü yavaşlattı ve açıyı 38'den 22'ye DÜŞÜRDÜ. Hız düşürülecekse
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
# TECS to arrest the climb. It used to be exactly FENCE_ALT_MAX, which put the
# whole run-in on the fence boundary and let a breach fight the 2 s
# DO_REPOSITION refresh for control of the vehicle.
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
# sonra araçta manevra değeriyle kalır (eski kodda TECS ailesi ve pitch tabanı
# bağlantı koptuğunda tam olarak böyle kalıyordu).
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
    # NOTE the different units -- writing 1500 to AIRSPEED_CRUISE would command
    # 1500 m/s. Old firmware ignores the new name and vice versa.
    ('AIRSPEED_CRUISE',  [(b'AIRSPEED_CRUISE', CRUISE_AIRSPEED_MS),
                          (b'TRIM_ARSPD_CM', CRUISE_AIRSPEED_MS * 100.0)]),
    ('NAVL1_PERIOD',     [(b'NAVL1_PERIOD', CRUISE_NAVL1_PERIOD)]),
]

# HSS güvenlik ağı + rally + AUTO seyrüsefer parametreleri. Bunlar manevralarla
# değişmediği için baseline'a dahil değil; bağlantı kurulduğunda bir kez yazılır.
HSS_SAFETY_PARAMS: list[tuple[str, list[tuple[bytes, float]]]] = [
    ('FENCE_ACTION',     [(b'FENCE_ACTION', FENCE_ACTION_RTL)]),
    ('FENCE_TYPE',       [(b'FENCE_TYPE', FENCE_TYPE_BITS)]),
    # Bit0=0: pilot/GCS fence ihlalinden sonra mod değiştirebilir. Kamikaze'nin
    # dışarıdan gelen mod değişimine boyun eğmesi (set_fly_mode) buna bağlı.
    ('FENCE_OPTIONS',    [(b'FENCE_OPTIONS', 0.0)]),
    ('FENCE_MARGIN',     [(b'FENCE_MARGIN', FENCE_MARGIN_M)]),
    ('FENCE_RET_RALLY',  [(b'FENCE_RET_RALLY', 1.0)]),
    ('FENCE_ALT_MAX',    [(b'FENCE_ALT_MAX', FENCE_ALT_MAX_M)]),
    ('FENCE_ALT_MIN',    [(b'FENCE_ALT_MIN', FENCE_ALT_MIN_M)]),
    ('RALLY_LIMIT_KM',   [(b'RALLY_LIMIT_KM', 0.0)]),
    ('RALLY_INCL_HOME',  [(b'RALLY_INCL_HOME', 1.0)]),
    ('MIS_RESTART',      [(b'MIS_RESTART', 0.0)]),
    ('WP_LOITER_RAD',    [(b'WP_LOITER_RAD', FENCE_LOITER_RADIUS_M)]),
    ('WP_MAX_RADIUS',    [(b'WP_MAX_RADIUS', 0.0)]),
    ('WP_RADIUS',        [(b'WP_RADIUS', 30.0)]),
    ('TKOFF_THR_MAX',    [(b'TKOFF_THR_MAX', 100.0)]),
    ('TKOFF_THR_MAX_T',  [(b'TKOFF_THR_MAX_T', TAKEOFF_FULL_THROTTLE_TIME)]),
]

class ParamOwner(Enum):
    """Araç durumunun o anki sahibi. Bkz. MainWindow.__set_param."""
    BASELINE = 0
    KAMIKAZE = 1

class RoutePlanSignals(QObject):
    # (RoutePlanResult | None). Lives on the GUI thread, so the connection from
    # the pool thread is automatically queued. Tipi object: hash değerleri 64
    # bit olabiliyor ve Qt'nin int'i 32 bit.
    finished = Signal(object, object)

    def __init__(self, parent):
        super().__init__(parent)

class RoutePlanTask(QRunnable):
    """
    compute_safe_route'u havuz thread'inde çalıştırır.

    Bu iş eskiden GUI thread'inde, HSS sinyalinin içinde senkron çalışıyordu.
    Görünürlük grafiği bölge sayısına göre karesel büyüdüğü için kalabalık bir
    HSS listesi GUI thread'ini saniyelerce kilitleyebiliyordu -- ve kamikaze'nin
    dalışı kesme kararı da (kamikaze_timer) aynı thread'de. Bir dalış sırasında
    o kilitlenme doğrudan yere çakılma demekti.
    """
    def __init__(self, waypoints: list, zones: list, plan_hash: int, signals: RoutePlanSignals):
        super().__init__()
        self._waypoints = waypoints
        self._zones = zones
        self._plan_hash = plan_hash
        self._signals = signals

    def run(self):
        result = None
        try:
            result = compute_safe_route(self._waypoints, self._zones)
        except Exception as e:
            qWarning("[RoutePreplanner] Rota hesabı başarısız: %s" % e)
        # Hata durumunda da sinyal atılıyor: atılmazsa _plan_in_flight sonsuza
        # kadar açık kalır ve bir daha hiç rota planlanmaz.
        self._signals.finished.emit(result, self._plan_hash)

class MavlinkWorkerSignals(QObject):
    set_fly_mode = Signal(int)
    set_arm_mode = Signal(int)
    should_reposition_removed = Signal()
    change_autopilot = Signal(int)

    def __init__(self, parent):
        super().__init__(parent)

class TrackableDataUpdate:
    @staticmethod
    def update_ground_speed(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_vfr_hud_message) -> str:
        telemetry.iha_hiz = packet.groundspeed
        return str(packet.groundspeed)
    @staticmethod
    def update_air_speed(packet: MAVLink_vfr_hud_message) -> str:
        return str(packet.airspeed)
    @staticmethod
    def update_velocity(packet: MAVLink_gps_raw_int_message) -> str:
        return str(packet.vel)
    @staticmethod
    def update_relative_altitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_global_position_int_message) -> str:
        telemetry.iha_irtifa = packet.relative_alt / 1000
        return str(packet.relative_alt / 1000)
    @staticmethod
    def update_altitude(packet: MAVLink_global_position_int_message) -> str:
        return str(packet.alt / 1000)
    @staticmethod
    def update_longitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_gps_raw_int_message) -> str:
        telemetry.iha_boylam = packet.lon / 1e7
        return str(packet.lon / 1e7)
    @staticmethod
    def update_latitude(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_gps_raw_int_message) -> str:
        telemetry.iha_enlem = packet.lat / 1e7
        return str(packet.lat / 1e7)
    @staticmethod
    def update_yaw(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        yaw: float = to_degree(packet.yaw)
        telemetry.iha_yonelme = yaw
        return str(yaw)
    @staticmethod
    def update_pitch(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        pitch: float = clamp(math.degrees(packet.pitch), -90, 90)
        telemetry.iha_dikilme = pitch
        return str(pitch)
    @staticmethod
    def update_roll(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_attitude_message) -> str:
        roll: float = clamp(math.degrees(packet.roll), -90, 90)
        telemetry.iha_yatis = roll
        return str(roll)
    @staticmethod
    def update_gps_time(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_system_time_message) -> str:
        datetime = QDateTime.fromMSecsSinceEpoch(int(packet.time_unix_usec / 1000))
        telemetry.gps_saati = GpsSaati(datetime)
        return datetime.toString()
    @staticmethod
    def update_battery_percentage(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_battery_status_message) -> str:
        telemetry.iha_batarya = packet.battery_remaining
        return str(packet.battery_remaining) + "%"
    @staticmethod
    def update_arm_status(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_heartbeat_message) -> str:
        arm: int = 1 if (packet.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0 else 0
        worker_signals.set_arm_mode.emit(arm)
        return str(arm != 0)

    @staticmethod
    def update_fly_mode(worker_signals: MavlinkWorkerSignals, telemetry: TelemetryData, packet: MAVLink_heartbeat_message) -> str:
        telemetry.iha_otonom = 1 if (packet.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) != 0 else 0
        worker_signals.change_autopilot.emit(packet.autopilot)
        if packet.autopilot == MAV_AUTOPILOT_PX4:
            sub_mod = packet.custom_mode >> 24
            base_mod = (packet.custom_mode >> 16) - (sub_mod << 8)
            px4mode = main_and_sub_mode_to_px4_uav_mode.get(base_mod)
            if px4mode:
                sub = px4mode.get(sub_mod)
                if sub:
                    index: int = sub.value[0]
                    worker_signals.set_fly_mode.emit(index)
                else:
                    qWarning("Can not handle px4 sub mode %s %s %s" % (packet.custom_mode, base_mod, sub_mod))
            else:
                qWarning("Can not handle px4 mode %s %s" % (packet.custom_mode, base_mod))
        elif packet.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
            index: int = packet.custom_mode
            if 27 > index >= 0:
                worker_signals.set_fly_mode.emit(index)
                if packet.custom_mode != 15:
                    worker_signals.should_reposition_removed.emit()
            else:
                qWarning("Don't know how to handle this custom mode data")
        else:
            qWarning("Unknown pilot type, fly mode unsupported")
        return ""

    @staticmethod
    def update_breach_status(packet: MAVLink_fence_status_message) -> str:
        text = "Breached"
        if packet.breach_status == 0:
            text = "Not " + text
        return text

TRACKABLE_DATA_ENUM_ACTIONS: dict[int, QAction] = {}

class TrackableDataEnum(Enum):
    # (id, name, update function, updater packet, should be updated on background (telemetry etc.), is it in watch_list widget, is it in watchlist at start)
    GROUND_SPEED = (0, lambda: QCoreApplication.translate("TrackableDataEnum", "Ground Speed", None), TrackableDataUpdate.update_ground_speed, TrackableDataPacketTimer.VFR_HUD, True, True, True)
    VELOCITY = (1, lambda: QCoreApplication.translate("TrackableDataEnum", "Velocity", None), TrackableDataUpdate.update_velocity, TrackableDataPacketTimer.GPS_RAW_INT, False, True, True)
    ALTITUDE = (2, lambda: QCoreApplication.translate("TrackableDataEnum", "Altitude", None), TrackableDataUpdate.update_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, False, True, False)
    YAW = (3, lambda: QCoreApplication.translate("TrackableDataEnum", "Yaw", None), TrackableDataUpdate.update_yaw, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    PITCH = (4, lambda: QCoreApplication.translate("TrackableDataEnum", "Pitch", None), TrackableDataUpdate.update_pitch, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    ROLL = (5, lambda: QCoreApplication.translate("TrackableDataEnum", "Roll", None), TrackableDataUpdate.update_roll, TrackableDataPacketTimer.ATTITUDE, True, True, True)
    AIR_SPEED = (6, lambda: QCoreApplication.translate("TrackableDataEnum", "Air Speed", None), TrackableDataUpdate.update_air_speed, TrackableDataPacketTimer.VFR_HUD, False, True, True)
    GPS_TIME = (7, lambda: QCoreApplication.translate("TrackableDataEnum", "GPS Time", None), TrackableDataUpdate.update_gps_time, TrackableDataPacketTimer.SYSTEM_TIME, True, True, True)
    LONGITUDE = (8, lambda: QCoreApplication.translate("TrackableDataEnum", "Longitude", None), TrackableDataUpdate.update_longitude, TrackableDataPacketTimer.GPS_RAW_INT, True, True, True)
    LATITUDE = (9, lambda: QCoreApplication.translate("TrackableDataEnum", "Latitude", None), TrackableDataUpdate.update_latitude, TrackableDataPacketTimer.GPS_RAW_INT, True, True, True)
    BATTERY_PERCENTAGE = (10, lambda: QCoreApplication.translate("TrackableDataEnum", "Battery Percentage", None), TrackableDataUpdate.update_battery_percentage, TrackableDataPacketTimer.BATTERY_STATUS, True, True, True)
    ARM_STATUS = (11, lambda: QCoreApplication.translate("TrackableDataEnum", "Arm Status", None), TrackableDataUpdate.update_arm_status, TrackableDataPacketTimer.HEARTBEAT, True, False, False)
    RELATIVE_ALTITUDE = (12, lambda: QCoreApplication.translate("TrackableDataEnum", "Relative Altitude", None), TrackableDataUpdate.update_relative_altitude, TrackableDataPacketTimer.GLOBAL_POSITION_INT, True, True, True)
    BREACH_STATUS = (13, lambda: QCoreApplication.translate("TrackableDataEnum", "Fence Breach Status", None), TrackableDataUpdate.update_breach_status, TrackableDataPacketTimer.FENCE_STATUS, False, True, False)
    FLY_MODE = (14, lambda: QCoreApplication.translate("TrackableDataEnum", "Fly Mode", None), TrackableDataUpdate.update_fly_mode, TrackableDataPacketTimer.HEARTBEAT, True, False, False)

    @staticmethod
    def from_id(i: int) -> TrackableDataEnum:
        e: TrackableDataEnum
        for e in TrackableDataEnum:
            if e.value[0] == i:
                return e
        return None

LANGUAGE_ACTIONS: dict[int, QAction] = {}

class UavConnection:
    connection_type: ConnectionType | None
    serial_port: str
    serial_baud_rate: int
    ip: str

    def __init__(self):
        self.connection_type = None
        self.serial_port = None
        self.serial_baud_rate = None
        self.ip = None

    def reset_connection_properties(self) -> None:
        self.serial_port = None
        self.serial_baud_rate = None
        self.ip = None
        self.connection_type = None

class ServerConnection:
    ip: str | None = None
    port: int = None
    username: str
    password: str
    team_no: int
    telemetry_timer: QTimer
    telemetry_thread: QThread = None

    def get_address(self) -> str:
        if self.port:
            return f"{self.ip}:{self.port}"
        return self.ip

class MavlinkWorker(QObject):
    parent: MainWindow
    running: bool
    update_watch_list = Signal(int, str)
    create_warning = Signal(str)
    connection_lost = Signal(str)
    fence_mission_count: int
    waypoint_mission_count: int
    mission_fence_item_received = Signal(int, float, float, int, float, int)
    mission_fence_item_int_received = Signal(int, float, float, int, float, int)
    mission_waypoint_item_received = Signal(int, float, float, float, int, int)
    mission_waypoint_item_int_received = Signal(int, float, float, float, int, int)
    send_fence_mission_data = Signal(int, bool)
    mission_upload_success = Signal(int)  # count
    mission_upload_failed = Signal(str)
    mission_current_changed = Signal(int)
    param_value_received = Signal(str, float)
    remove_reposition_location = Signal()
    worker_signals: MavlinkWorkerSignals
    _mission_last_activity_time: float

    def __init__(self, mavlink_connection: mavfile, parent: MainWindow):
        super().__init__()
        self.mavlink_connection = mavlink_connection
        self.parent = parent
        self.watch_list = parent.ui.watch_list
        self.running = False
        self.fence_mission_count = 0
        self.waypoint_mission_count = 0
        self.worker_signals = MavlinkWorkerSignals(self)
        
        # Async Mission Upload State
        self._mission_upload_items = []
        self._mission_upload_state = 0 # 0: IDLE, 1: CLEARING, 2: COUNTING, 3: UPLOADING
        self._start_mission_upload = False
        self._mission_last_activity_time = 0.0
        self._mission_upload_retry_count = 0
        self._mission_upload_current_seq = 0

    def _send_mission_clear_all(self) -> None:
        self.mavlink_connection.mav.mission_clear_all_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_MISSION_TYPE_MISSION
        )
        qDebug("[MavlinkWorker] Sent MISSION_CLEAR_ALL")

    def _send_mission_count(self) -> None:
        self.mavlink_connection.mav.mission_count_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            len(self._mission_upload_items),
            MAV_MISSION_TYPE_MISSION
        )
        qDebug(f"[MavlinkWorker] Sent MISSION_COUNT: {len(self._mission_upload_items)}")

    def _send_mission_item_int(self, seq: int) -> None:
        if seq >= len(self._mission_upload_items):
            return
        coord = self._mission_upload_items[seq]
        alt = coord.altitude()
        from pymavlink.dialects.v20.all import MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_NAV_WAYPOINT
        frame_int = MAV_FRAME_GLOBAL_INT if seq == 0 else MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        self.mavlink_connection.mav.mission_item_int_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            seq,
            frame_int,
            MAV_CMD_NAV_WAYPOINT,
            0,   # current
            1,   # autocontinue
            0,   # param1
            5.0, # param2
            0,   # param3
            float('nan'), # param4
            int(coord.latitude() * 1e7),
            int(coord.longitude() * 1e7),
            alt,
            MAV_MISSION_TYPE_MISSION
        )
        qDebug(f"[MavlinkWorker] Sent MISSION_ITEM_INT for seq {seq}")

    def trigger_update_value(self, e: TrackableDataEnum, packet: MAVLink_message):
        length: int = self.watch_list.rowCount()
        new_val: str | None = None
        if e.value[4]:
            self.parent.next_telemetry.lock.lockForWrite()
            try:
                new_val = e.value[2](self.worker_signals, self.parent.next_telemetry, packet)  # Update if it is telemetry without caring it is in watch list or not
            finally:
                self.parent.next_telemetry.lock.unlock()
        for i in range(length):
            if self.watch_list.item(i, 0).text() == str(e.value[0]):
                if new_val is None:
                    new_val = e.value[2](packet)
                self.update_watch_list.emit(i, new_val)
                break

    def run(self):
        while self.running:
            if self._start_mission_upload:
                self._start_mission_upload = False
                self._mission_upload_state = 1 # CLEARING
                self._mission_upload_retry_count = 0
                self._mission_upload_current_seq = 0
                self._send_mission_clear_all()
                self._mission_last_activity_time = time.monotonic()

            if self._mission_upload_state > 0:
                if time.monotonic() - self._mission_last_activity_time > 2.0:
                    if self._mission_upload_state <= 0:
                        return
                    self._mission_upload_retry_count += 1
                    if self._mission_upload_retry_count > 3:
                        self._mission_upload_state = 0
                        self.mission_upload_failed.emit("Rota yükleme zaman aşımına uğradı (Otopilot cevap vermedi)")
                    else:
                        qWarning(
                            f"[MavlinkWorker] Timeout at state {self._mission_upload_state}, retrying ({self._mission_upload_retry_count}/3)")
                        if self._mission_upload_state == 1:
                            self._send_mission_clear_all()
                        elif self._mission_upload_state == 2:
                            self._send_mission_count()
                        elif self._mission_upload_state == 3:
                            self._send_mission_item_int(self._mission_upload_current_seq)

            timeout_val = 0.1 if self._mission_upload_state > 0 else 1.0
            try:
                packet: MAVLink_message = self.mavlink_connection.recv_match(blocking=True, timeout=timeout_val)
            except OSError as e:
                # Serial port died (USB glitch, EMI, unplug); the handle is
                # invalid from here on, so stop reading and tell the GUI.
                if self.running:
                    qWarning("MAVLink connection I/O error: %s" % e)
                    self.connection_lost.emit(str(e))
                break
            except Exception as e:
                qWarning("Error while reading MAVLink packet: %s" % e)
                continue
            if not self.running:
                break
            if packet is None:
                continue
            msgID: int = packet.get_header().msgId
            if msgID == MAVLINK_MSG_ID_BAD_DATA:
                if all_printable(packet.data):
                    qWarning("Invalid data received: %s" % packet.data)
                else:
                    qWarning("Invalid data received")
            elif msgID in (MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAVLINK_MSG_ID_MISSION_REQUEST):
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.send_fence_mission_data.emit(packet.seq, msgID == MAVLINK_MSG_ID_MISSION_REQUEST_INT)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION and self._mission_upload_state > 0:
                    if self._mission_upload_state == 1:
                        # ArduPilot skipped CLEAR_ALL ack or we missed it, assume it proceeded
                        self._mission_upload_state = 2
                    if self._mission_upload_state == 2:
                        self._mission_upload_state = 3 # Transition to UPLOADING
                    self._mission_upload_retry_count = 0
                    self._mission_last_activity_time = time.monotonic()
                    self._mission_upload_current_seq = packet.seq
                    self._send_mission_item_int(packet.seq)
            elif msgID == MAVLINK_MSG_ID_MISSION_ACK:
                qDebug("MissionACK packet received with type %s and with mission_type %s" % (packet.type, packet.mission_type))
                if packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    if self._mission_upload_state == 1:
                        if packet.type == MAV_MISSION_ACCEPTED:
                            self._mission_upload_state = 2 # Proceed to COUNTING
                            self._mission_upload_retry_count = 0
                            self._send_mission_count()
                            self._mission_last_activity_time = time.monotonic()
                        else:
                            self._mission_upload_state = 0
                            self.mission_upload_failed.emit(f"Görev temizleme reddedildi (ACK: {packet.type})")
                    elif self._mission_upload_state == 3:
                        self._mission_upload_state = 0 # Upload finished
                        if packet.type == MAV_MISSION_ACCEPTED:
                            self.mission_upload_success.emit(len(self._mission_upload_items))
                        else:
                            self.mission_upload_failed.emit(f"Otopilot rotayı reddetti (ACK: {packet.type})")
            elif msgID == MAVLINK_MSG_ID_MISSION_COUNT:
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    if not self.parent.requested_to_get_fence:
                        continue
                    self.fence_mission_count = packet.count
                    qDebug("Received %s fence point" % self.fence_mission_count)
                    if self.fence_mission_count > 0:
                        self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, 0, MAV_MISSION_TYPE_FENCE)
                    else:
                        self.parent.requested_to_get_fence = False
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    if not self.parent.requested_to_get_mission:
                        continue
                    self.waypoint_mission_count = packet.count
                    qDebug("Received %s mission waypoint" % self.waypoint_mission_count)
                    if self.waypoint_mission_count > 0:
                        self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, 0, MAV_MISSION_TYPE_MISSION)
                    else:
                        self.parent.requested_to_get_mission = False

            elif msgID == MAVLINK_MSG_ID_MISSION_ITEM_INT:
                packet: MAVLink_mission_item_int_message = packet
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_int_received.emit(packet.command, packet.x / 1e7, packet.y / 1e7, packet.seq, packet.param1, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_int_received.emit(packet.command, packet.x / 1e7, packet.y / 1e7, packet.z, packet.seq, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_MISSION_ITEM:
                packet: MAVLink_mission_item_message = packet
                if packet.mission_type == MAV_MISSION_TYPE_FENCE:
                    self.mission_fence_item_received.emit(packet.command, packet.x, packet.y, packet.seq, packet.param1, self.fence_mission_count)
                elif packet.mission_type == MAV_MISSION_TYPE_MISSION:
                    self.mission_waypoint_item_received.emit(packet.command, packet.x, packet.y, packet.z, packet.seq, self.waypoint_mission_count)
            elif msgID == MAVLINK_MSG_ID_COMMAND_ACK:
                command: int = packet.command
                result: int = packet.result

                if command == MAV_CMD_DO_REPOSITION:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Reposition command successfully executed")
                    elif result == MAV_RESULT_DENIED:
                        self.create_warning.emit("Reposition command denied by vehicle")
                        self.remove_reposition_location.emit()
                    else:
                        qDebug("Reposition result: %s" % result)
                elif command == MAV_CMD_DO_SET_MODE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Set mode command successfully executed")
                    elif result == MAV_RESULT_TEMPORARILY_REJECTED:
                        self.create_warning.emit("Set mode temporarily rejected by vehicle, sensor/calibration error")
                    else:
                        qDebug("Set mode result: %s" % result)
                elif command == MAV_CMD_SET_MESSAGE_INTERVAL:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Message interval successfully set")
                    elif result == MAV_RESULT_FAILED:
                        qWarning("Couldn't set some message interval")
                    else:
                        qDebug("Set message interval result: %s" % result)
                elif command == MAV_CMD_REQUEST_MESSAGE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Message request accepted")
                    else:
                        qDebug("Message request result: %s" % result)
                elif command == MAV_CMD_DO_FENCE_ENABLE:
                    if result == MAV_RESULT_ACCEPTED:
                        qDebug("Fence enable command accepted")
                    else:
                        qDebug("Fence enable command result: %s" % result)
                else:
                    qDebug("CommandACK received for command %s and result %s" % (command, result))
            elif msgID == MAVLINK_MSG_ID_MISSION_CURRENT:
                self.mission_current_changed.emit(packet.seq)
            elif msgID == MAVLINK_MSG_ID_PARAM_VALUE:
                # ArduPilot her PARAM_SET'e PARAM_VALUE ile cevap verir; bu
                # MainWindow'daki bekleyen yazıların teyidi (bkz. __set_param).
                param_id = packet.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode('ascii', 'replace')
                self.param_value_received.emit(param_id.rstrip('\x00'), float(packet.param_value))
            elif msgID in MSG_ID_2_TRACKABLE_DATA_TYPE:
                e = MSG_ID_2_TRACKABLE_DATA_TYPE[msgID]
                data_enum_values = e.value[4]
                for i in data_enum_values:
                    self.trigger_update_value(TrackableDataEnum.from_id(i), packet)
            # else:
            #     qDebug("Ignoring packet with id %s" % msgID)

class ConnectionWaitWrapper(QObject):
    after_heartbeat_successfully_received = Signal(mavfile)
    setup_for_autopilot = Signal(int)
    after_heartbeat_not_received = Signal(mavfile)
    mavlink_connection_error = Signal()
    set_device_connection_text = Signal(str)
    mavlink_connection: mavfile
    uav_connection: UavConnection
    __parent: MainWindow
    con_thread: QThread

    def __init__(self, parent: MainWindow, uav_connection: UavConnection):
        super().__init__()
        self.uav_connection = uav_connection
        self.mavlink_connection = None
        self.__parent = parent

    def run(self):
        self.set_device_connection_text.emit(QCoreApplication.translate("UAVConnection", "Trying to connect device :O", None))
        try:
            match self.uav_connection.connection_type:
              case ConnectionType.TCP:
                  self.mavlink_connection = mavtcp(self.uav_connection.ip, retries=1)
              case ConnectionType.UDP:
                  self.mavlink_connection = mavudp(self.uav_connection.ip, timeout=10)
              case ConnectionType.SERIAL:
                  self.mavlink_connection = mavserial(self.uav_connection.serial_port, baud=self.uav_connection.serial_baud_rate)
              case None:
                  self.mavlink_connection_error.emit()
                  qWarning("Connection type is null ???")
                  return
        except OSError as e:
            self.set_device_connection_text.emit(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))
            qWarning("Tried a invalid connection: %s" % str(e))
            self.mavlink_connection_error.emit()
            return
        qInfo("Successfully Connected with mavlink, Target System: %s, Target component: %s" % (self.mavlink_connection.target_system, self.mavlink_connection.target_component))

        try:
            msg: MAVLink_heartbeat_message = self.mavlink_connection.wait_heartbeat(timeout=10)
            if msg is None:
                raise Exception("Connection failed")

            qInfo("Successfully Received first heartbeat")
            self.setup_for_autopilot.emit(msg.autopilot)
            self.after_heartbeat_successfully_received.emit(self.mavlink_connection)
        except:
            self.after_heartbeat_not_received.emit(self.mavlink_connection)
        self.__parent.connection_wait_wrapper = None
        self.con_thread.quit()

class NoAccentStyle(QProxyStyle):
    def __init__(self, style: QStyle):
        super().__init__(style)

    def drawPrimitive(self, element, option, painter, /, widget=...):
        if element == QStyle.PrimitiveElement.PE_PanelItemViewRow:
            return

        super().drawPrimitive(element, option, painter, widget)


class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    uav_connection: UavConnection = UavConnection()
    uav_connection_dialog: FightingUAVConnectionInterface | None = None
    server_connection_dialog: ServerConnectionInterface | None = None
    camera_server_connection_dialog: CameraServerConnectionInterface | None = None
    color_selector_dialog: ColorSelectorInterface | None = None
    input_selector_dialog: KeybindingConfigInterface | None = None
    geofence_dialog: SetGeofenceInterface | None = None
    add_ads_dialog: AddADSInterface | None = None
    server_connection: ServerConnection = ServerConnection()
    mavlink_connection: mavfile = None
    color_options: ColorOptions
    mavlink_worker: MavlinkWorker | None = None
    mavlink_thread: QThread | None = None
    next_telemetry: TelemetryData = TelemetryData()
    last_server_telemetry_response: TelemetryResponseData = TelemetryResponseData()
    plane_on_map_update_timer: QTimer = QTimer(interval=500)
    current_lang: int
    current_pilot: int
    _hss_polling_worker: HSSPollingWorker | None = None
    _hss_polling_thread: QThread | None = None
    _current_snapshot: HssSnapshot | None = None
    _current_mission_waypoints: list = []
    _pixhawk_current_seq: int = 0
    _last_planned_hash = None
    # Kamikaze koşusu aracın sahibiyken gelen HSS anlık görüntüsü buraya
    # bırakılır ve koşu biter bitmez uygulanır. Bkz. _on_hss_updated.
    _deferred_snapshot: HssSnapshot | None = None
    _deferred_manual_ads: bool = False
    _plan_in_flight: bool = False
    _fence_upload_retries: int = 0
    # Doğrulama amaçlı görev indirmesi rota planlamasını yeniden tetiklemesin.
    _awaiting_mission_verification: bool = False
    _param_pending: dict
    _param_retry_timer: QTimer
    _route_plan_signals: RoutePlanSignals
    _kamikaze_run_started_at: float = 0.0
    uav_is_armed: bool = False
    _mode_change_cooldown: QTimer
    _arm_change_cooldown: QTimer
    _gcs_heartbeat_timer: QTimer
    _kamikaze_target_cooldown: QTimer

    def __init__(self):
        QMainWindow.__init__(self)
        self.current_lang = 0
        self.translator = QTranslator()
        self.setStyle(NoAccentStyle(self.style()))
        self.color_options = ColorOptions()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_colors()

        self.kamikaze_state = KamikazeState.IDLE
        self.kamikaze_original_alt = 0.0
        self.kamikaze_target_lat = 0.0
        self.kamikaze_target_lon = 0.0
        self.kamikaze_previous_mode = None
        self.kamikaze_timer = QTimer(self)
        self.kamikaze_timer.setInterval(KAMIKAZE_TICK_INTERVAL)
        self.kamikaze_timer.timeout.connect(self.__kamikaze_loop)
        self.kamikaze_recover_heading = 0.0
        self.kamikaze_alt_history = []
        self.kamikaze_dive_sink_max = 0.0
        self.kamikaze_lowest_alt = math.inf
        self.kamikaze_steepest_angle = 0.0
        self.kamikaze_steepest_pitch = 0.0
        self.kamikaze_sink_peaks = []
        self.kamikaze_dive_started_at = 0.0
        self.kamikaze_dive_start_alt = 0.0
        self.kamikaze_dive_duration = 0.0
        self._kamikaze_climb_warned = False
        self._kamikaze_target_cooldown = QTimer(self, singleShot=True, interval=KAMIKAZE_TARGET_REFRESH)
        self.waits_for_qr = False
        self._param_pending = {}
        self._param_retry_timer = QTimer(self, interval=PARAM_ACK_TIMEOUT)
        self._param_retry_timer.timeout.connect(self.__retry_pending_params)
        self._route_plan_signals = RoutePlanSignals(self)
        self._route_plan_signals.finished.connect(self._on_route_plan_ready)
        self._mode_change_cooldown = QTimer(self, singleShot=True, interval=2000)
        self._arm_change_cooldown = QTimer(self, singleShot=True, interval=2000)
        self._gcs_heartbeat_timer = QTimer(self, interval=1000)
        self._gcs_heartbeat_timer.timeout.connect(self.__send_gcs_heartbeat)

        self.current_pilot = MAV_AUTOPILOT_INVALID

        self.ui.actionConfigurate_UAV.triggered.connect(self._actionConfigurate_UAV)
        self.ui.actionConfigurateServer.triggered.connect(self._actionConfigurateServer)
        self.ui.actionSet_Colors.triggered.connect(self._actionConfigurateSetColors)

        add_to_watch_menu: QMenu = QMenu(parent=self)

        for e in TrackableDataEnum:
            action: QAction = QAction(text=e.value[1](), parent=self)
            action.setObjectName(str(e.value[0]))
            action.triggered.connect(partial(self.add_to_watch_list, e))
            TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]] = action
            if e.value[5]:
                add_to_watch_menu.addAction(action)

        self.ui.add_to_watch.setMenu(add_to_watch_menu)

        for e in TrackableDataEnum:
            if e.value[5] and e.value[6]:
                self.add_to_watch_list(e)

        for lang in SupportedLanguages:
            lang_id = lang.value[0]
            action: QAction = QAction(lang.value[1](), self)
            action.setObjectName(str(lang_id))
            action.setCheckable(True)
            action.triggered.connect(partial(self.change_lang_to, lang_id))
            LANGUAGE_ACTIONS[lang_id] = action
            self.ui.menuChange_Language.addAction(action)
        LANGUAGE_ACTIONS[0].setChecked(True)

        self.ui.remove_from_watch.clicked.connect(self.__remove_from_watch_signal)
        self.ui.watch_list.setColumnHidden(0, True) # hide id column
        self.ui.fly_mode_combobox.activated.connect(self._change_index)
        self.ui.get_kamikaze_coords_from_server.clicked.connect(self.__get_kamikaze_coords)
        self.ui.start_kamikaze.clicked.connect(self.__start_kamikaze)
        self.ui.force_end_task.clicked.connect(self.__force_end_task)
        self.ui.set_fence.clicked.connect(self.__set_fence_clicked)
        self.ui.arm_mode.activated.connect(self.__setArmStatus)
        self.ui.refresh_ads.clicked.connect(self.__refresh_ads)
        self.ui.add_ads.clicked.connect(self.__add_ads_button_clicked)
        self.plane_on_map_update_timer.timeout.connect(self.__update_plane_on_map_without_server)
        self.ui.start_stop_camera_view.toggled.connect(self.__start_stop_camera_view)
        self.ui.actionConfigurate_Camera_Stream.triggered.connect(self._actionConfigurateCameraServer)
        self.ui.remove_ads.clicked.connect(self._remove_ads)
        self.ui.map_view.coords_for_geofence.upload_geofence_data.connect(lambda: self.update_geofence_data(self.ui.map_view.server_ads_data_model.m_datas + self.ui.map_view.user_ads_data_model.m_datas))
        self.ui.map_view.upload_ads_data.connect(self._on_manual_ads_changed)
        self.ui.actionAbout.triggered.connect(self._about)
        self.ui.actionAbout_Qt.triggered.connect(lambda: QMessageBox.aboutQt(self))
        self.fence_upload_timout = QTimer(self, singleShot=True, interval=10000)
        self.fence_upload_timout.timeout.connect(self.fence_upload_reset)
        self.fence_download_timout = QTimer(self, singleShot=True, interval=10000)
        self.fence_download_timout.timeout.connect(self.request_fence_data_timeout)
        self.mission_download_timout = QTimer(self, singleShot=True, interval=10000)
        self.mission_download_timout.timeout.connect(self.request_mission_data_timeout)
        self.ui.download_missions.clicked.connect(self.request_mission_data)
        self.ui.download_fence_data.clicked.connect(self.request_fence_data)
        self.input_config = KeybindingConfigInterface.initialize_mappings()
        self.ui.map_view.set_input_config_reference(lambda: self.input_config)
        self.ui.actionChange_Input_Mapping.triggered.connect(self.__open_input_map_config_dialog)
        floatValidator: QDoubleValidator = QDoubleValidator(parent=self)
        floatValidator.setNotation(QDoubleValidator.Notation.StandardNotation)
        self.ui.kamikaze_latitude.setValidator(floatValidator)
        self.ui.kamikaze_longitude.setValidator(floatValidator)
        self.update_plane_data_signal.connect(self.__update_plane_data)
        self.ui.camera_view.qr_successfully_readed.connect(self.on_qr_found)
        self.ui.disable_enable_locking.toggled.connect(self.ui.camera_view.change_lock_state)
        self.ui.camera_view.set_mainwindow_reference(self)

    def setup_colors(self):
        self.setStyleSheet(ColorSelectorInterface.create_stylesheet(self.color_options))

    def change_autopilot(self, new_autopilot: int):
        if self.current_pilot == new_autopilot:
            return
        self.current_pilot = new_autopilot
        if new_autopilot == MAV_AUTOPILOT_PX4:
            self.ui.fly_mode_combobox.clear()
            for mode in PX4_UAV_Modes:
                self.ui.fly_mode_combobox.insertItem(mode.value[0], mode.value[1])
                if mode.value[2] == 0:
                    self.ui.fly_mode_combobox.view().setRowHidden(mode.value[0], True)
        elif new_autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.ui.fly_mode_combobox.clear()
            for mode in Ardupilot_UAV_Modes:
                self.ui.fly_mode_combobox.insertItem(mode.value[0], mode.value[1])
                if mode.value[2] == -1:
                    self.ui.fly_mode_combobox.view().setRowHidden(mode.value[0], True)
        self.ui.fly_mode_combobox.setCurrentIndex(-1)

    input_config: dict[KeybindingsEnum, InputMapping]
    def __open_input_map_config_dialog(self):
        if self.input_selector_dialog is not None:
            return
        self.input_selector_dialog = KeybindingConfigInterface(self, self.input_config)
        self.input_selector_dialog.finished.connect(self._input_map_close_screen)
        self.input_selector_dialog.show()

    def _input_map_close_screen(self):
        self.input_selector_dialog = None

    def closeEvent(self, event, /):
        try:
            self._uav_disconnect()
        except:
            pass
        try:
            self._server_disconnect()
        except:
            pass
        super().closeEvent(event)

    next_mission_order_seq_id: int = 0
    requested_to_get_mission: bool = False
    def request_mission_data(self):
        if self.mavlink_connection is None:
            return
        if self.requested_to_get_mission:
            qDebug("Tried to get missions when already getting missions from uav")
            return
        self.requested_to_get_mission = True
        self.mavlink_connection.mav.mission_request_list_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_MISSION)
        self.mission_download_timout.start()

    def request_mission_data_timeout(self):
        self._create_warning("Mission Download taking really long, probably vehicle connection has been lost")
        self.next_mission_order_seq_id = 0
        self.requested_to_get_mission = False
        self._awaiting_mission_verification = False
        self.ui.map_view.mission_coords_data_model.layoutChanged.emit()
        self.ui.map_view.mission_geopath.mission_geopath_changed.emit()

    def mission_waypoint_received(self, coord: QGeoCoordinate, command: int, seq: int, use_int: bool, count: int):
        if not self.requested_to_get_mission or count == 0:
            return
        qDebug("Mission received with %s coords, %s command, %s seq" % (coord, command, seq))
        if self.next_mission_order_seq_id != seq:
            qDebug("Out of order mission")
            return
        if seq == 0:
            self.ui.map_view.mission_coords_data_model.m_datas.clear()
            self.ui.map_view.mission_geopath.clear()
        coord_data: SpecialCoordsData = SpecialCoordsData()
        coord_data.position = coord
        coord_data.coord_type = 1
        self.ui.map_view.mission_coords_data_model.m_datas.append(coord_data)
        self.ui.map_view.mission_geopath.add_pos(coord)
        self.mission_download_timout.start()
        if seq + 1 != count:
            if use_int:
                self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_MISSION)
            else:
                self.mavlink_connection.mav.mission_request_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_MISSION)
            self.next_mission_order_seq_id = seq + 1
        else:
            self.mission_download_timout.stop()
            self.mavlink_connection.mav.mission_ack_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION)
            self.next_mission_order_seq_id = 0
            self.requested_to_get_mission = False
            self.ui.map_view.mission_coords_data_model.layoutChanged.emit()
            self.ui.map_view.mission_geopath.mission_geopath_changed.emit()

            # HSS entegrasyonu: indirilen mission kaydedilir, rota düzeltme tetiklenir
            # Outlier filtreleme: (0.0, 0.0) koordinatlı noktaları rota planlamasından çıkar
            all_positions = [d.position for d in self.ui.map_view.mission_coords_data_model.m_datas]
            self._current_mission_waypoints = []
            for i, pos in enumerate(all_positions):
                if abs(pos.latitude()) < 0.01 and abs(pos.longitude()) < 0.01:
                    qDebug("[Mission] Filtering outlier WP %d at (%.4f, %.4f)" % (i, pos.latitude(), pos.longitude()))
                    continue
                self._current_mission_waypoints.append(pos)
            if self._awaiting_mission_verification:
                # Bu indirme, az önce yüklediğimiz düzeltilmiş rotanın teyidi.
                # Yeniden planlamayı tetiklemek yükle→indir→planla→yükle
                # döngüsüne yol açardı.
                self._awaiting_mission_verification = False
                self._create_warning("Rota doğrulandı: araçta %d waypoint var"
                                     % len(self._current_mission_waypoints))
                qDebug("[MissionUpload] Verification download complete: %d waypoints"
                       % len(self._current_mission_waypoints))
                return
            # Hash sıfırla — yeni mission'da HSS değişmemiş olsa bile rota yeniden hesaplansın
            self._last_planned_hash = None
            self._trigger_route_replanning()

    def mission_waypoint_item_received(self, command: int, x: float, y: float, z: float, seq: int, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y, z)
        self.mission_waypoint_received(coord, command, seq, False, count)

    def mission_waypoint_item_int_received(self, command: int, x: float, y: float, z: float, seq: int, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y, z)
        self.mission_waypoint_received(coord, command, seq, True, count)

    def request_fence_data(self):
        if self.mavlink_connection is None:
            return
        if self.requested_to_get_fence:
            qDebug("Tried to get fence when already getting fence from uav")
            return
        self.requested_to_get_fence = True
        self.coord_counter = 1
        self.next_fence_order_seq_id = 0
        self.mavlink_connection.mav.mission_request_list_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_FENCE)
        self.fence_download_timout.start()

    def request_fence_data_timeout(self):
        self._create_warning("Fence Download taking really long, probably vehicle connection has been lost")
        self.next_fence_order_seq_id = 0
        self.requested_to_get_fence = False
        self.ui.map_view.user_ads_data_model.layoutChanged.emit()
        self.coord_counter = 1

    coord_counter: int = 1
    next_fence_order_seq_id: int = 0
    requested_to_get_fence: bool = False
    def mission_fence_item_int_received(self, command: int, x: float, y: float, seq: int, ads_size: float, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_fence_received(coord, command, seq, ads_size, True, count)

    # TODO: This handling assumes 4 vertex fence
    def mission_fence_received(self, coord: QGeoCoordinate, command: int, seq: int, ads_size: float, use_int: bool, count: int):
        if not self.requested_to_get_fence or count == 0:
            return
        qDebug("Fence received with %s coords, %s command, %s seq" % (coord, command, seq))
        if self.next_fence_order_seq_id != seq:
            qDebug("Out of order fence")
            return
        if command == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            if seq == 0:
                self.ui.map_view.user_ads_data_model.m_datas.clear()
            ads = AdsData()
            ads.position = coord
            ads.size = ads_size
            ads.is_selected = False
            self.ui.map_view.user_ads_data_model.m_datas.append(ads)
        elif command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            data: SpecialCoordsData = SpecialCoordsData()
            data.position = coord
            data.coord_type = self.coord_counter + 5  # I hope i will remember how this works
            match self.coord_counter:
                case 1:
                    self.ui.map_view.coord_data_model.m_datas.clear()
                    self.ui.map_view.coords_for_geofence.gc1_v = coord
                case 2:
                    self.ui.map_view.coords_for_geofence.gc2_v = coord
                case 3:
                    self.ui.map_view.coords_for_geofence.gc3_v = coord
                case 4:
                    self.ui.map_view.coords_for_geofence.gc4_v = coord
                    self.ui.map_view.coords_for_geofence.gc_changed.emit()
                    self.coord_counter = 0
            self.ui.map_view.coord_data_model.m_datas.append(data)
            self.ui.map_view.coord_data_model.layoutChanged.emit()
            self.coord_counter += 1
        self.fence_download_timout.start()
        if seq + 1 != count:
            if use_int:
                self.mavlink_connection.mav.mission_request_int_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_FENCE)
            else:
                self.mavlink_connection.mav.mission_request_send(self.mavlink_connection.target_system,
                                                                     self.mavlink_connection.target_component, seq + 1,
                                                                     MAV_MISSION_TYPE_FENCE)
            self.next_fence_order_seq_id = seq + 1
        else:
            self.fence_download_timout.stop()
            self.mavlink_connection.mav.mission_ack_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE)
            self.next_fence_order_seq_id = 0
            self.requested_to_get_fence = False
            self.ui.map_view.user_ads_data_model.layoutChanged.emit()

    def mission_fence_item_received(self, command: int, x: float, y: float, seq: int, ads_size: float, count: int):
        coord: QGeoCoordinate = QGeoCoordinate(x, y)
        self.mission_fence_received(coord, command, seq, ads_size, False, count)

    def _about(self):
        QMessageBox.about(self, QCoreApplication.translate("MainWindow", u"About", None), QCoreApplication.translate("MainWindow", "Designed and developed by Muzaffer Kadir Belen to be used by ARES teknofest team", None))

    def _remove_ads(self):
        self.ui.map_view.mouse_input_handler.remove_selected_ads(None, 0, 0)

    def _actionConfigurateCameraServer(self):
        if self.camera_server_connection_dialog is not None:
            return
        self.camera_server_connection_dialog = CameraServerConnectionInterface(self)
        for protocol in CameraServerProtocol:
            self.camera_server_connection_dialog.ui.server_protocol_type.insertItem(protocol.value[0], protocol.value[1])
        self.camera_server_connection_dialog.show()
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Connected :)", None))
            self.camera_server_connection_dialog.ui.server_ip_input.setText(self.ui.camera_view.camera_server_info.ip)
            self.camera_server_connection_dialog.ui.camera_width.setText(str(self.ui.camera_view.camera_server_info.width))
            self.camera_server_connection_dialog.ui.camera_height.setText(str(self.ui.camera_view.camera_server_info.height))
            self.camera_server_connection_dialog.ui.server_protocol_type.setCurrentIndex(self.ui.camera_view.camera_server_info.protocol.value[0])
        self.camera_server_connection_dialog.ui.connect.clicked.connect(self.connect_to_cam_server)
        self.camera_server_connection_dialog.ui.disconnect.clicked.connect(self.disconnect_from_cam_server)
        self.camera_server_connection_dialog.finished.connect(self.reset_camera_server_dialog)

    def connect_to_cam_server(self):
        if self.ui.camera_view.camera_server_info.ip is not None:
            self.ui.camera_view.disconnect_from_server()
        if not self.camera_server_connection_dialog.ui.camera_height.text():
            self.camera_server_connection_dialog.ui.camera_height.setText(self.camera_server_connection_dialog.ui.camera_height.placeholderText())
        if not self.camera_server_connection_dialog.ui.camera_width.text():
            self.camera_server_connection_dialog.ui.camera_width.setText(self.camera_server_connection_dialog.ui.camera_width.placeholderText())
        ip_and_port = self.camera_server_connection_dialog.ui.server_ip_input.text()
        if ":" in ip_and_port:
            ip_and_port = ip_and_port.split(":")
            if len(ip_and_port) != 2:
                self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
                return
            else:
                ip: str = ip_and_port[0]
                port: int
                try:
                    port = int(ip_and_port[1])
                except ValueError:
                    self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
                    return
                self.ui.camera_view.camera_server_info.ip = ip
                self.ui.camera_view.camera_server_info.port = port
        self.ui.camera_view.camera_server_info.width = int(self.camera_server_connection_dialog.ui.camera_width.text())
        self.ui.camera_view.camera_server_info.height = int(self.camera_server_connection_dialog.ui.camera_height.text())
        self.ui.camera_view.camera_server_info.recalculate_frame_size()
        self.ui.camera_view.set_protocol(self.camera_server_connection_dialog.ui.server_protocol_type.currentIndex())
        if self.ui.camera_view.connect_to_server():
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Connected :)", None))
            self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(False)
            self.ui.start_stop_camera_view.setCheckable(True)
            self.ui.record_button.setCheckable(True)
        else:
            self.camera_server_connection_dialog.ui.camera_connection_text.setText(QCoreApplication.translate("CameraConfig", "Camera Not Connected :(", None))
            self.camera_server_connection_dialog.ui.invalid_input_error_label.setEnabled(True)
            self.ui.start_stop_camera_view.setCheckable(False)
            self.ui.record_button.setCheckable(False)

    def disconnect_from_cam_server(self):
        self.ui.camera_view.disconnect_from_server()
        self.ui.camera_view.set_no_connection_image()
        self.ui.camera_view.camera_server_info.ip = None
        self.ui.start_stop_camera_view.setCheckable(False)
        self.ui.record_button.setCheckable(False)

    def reset_camera_server_dialog(self):
        self.camera_server_connection_dialog = None

    def __start_stop_camera_view(self, b: bool) -> None:
        if b:
            self.ui.camera_view.on_play()
        else:
            self.ui.camera_view.on_pause()

    def retranslateWatcher(self):
        length: int = self.ui.watch_list.rowCount()
        for i in range(length):
            tde = TrackableDataEnum.from_id(int(self.ui.watch_list.item(i, 0).text()))

            self.ui.watch_list.setItem(i, 1, QTableWidgetItem(tde.value[1]()))
            if self.mavlink_connection is None:
                self.ui.watch_list.setItem(i, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))
        for e in TRACKABLE_DATA_ENUM_ACTIONS.values():
            tde = TrackableDataEnum.from_id(int(e.objectName()))

            e.setText(tde.value[1]())

    def resetWatcherWidgetValues(self):
        length: int = self.ui.watch_list.rowCount()
        for i in range(length):
            self.ui.watch_list.setItem(i, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))

    def get_all_dialogs(self):
        return [self.uav_connection_dialog,
                   self.server_connection_dialog,
                   self.color_selector_dialog,
                   self.geofence_dialog,
                   self.add_ads_dialog,
                   self.camera_server_connection_dialog,
                   self.ui.map_view.mouse_input_handler.ard_dialog,
                   self.input_selector_dialog]

    def retranslateOpenDialogs(self):
        dialogs = self.get_all_dialogs()

        for dialog in dialogs:
            if dialog is not None:
                dialog.ui.retranslateUi(dialog)

        if self.color_selector_dialog is not None:
            for dialog in self.color_selector_dialog.get_dialogs():
                dialog.retranslateUi()

    translator: QTranslator
    def change_lang_to(self, index: int):
        slang = SupportedLanguages.from_id(index)
        LANGUAGE_ACTIONS[self.current_lang].setChecked(False)
        self.current_lang = index
        locale: QLocale = QLocale(slang.value[2], slang.value[3])
        QLocale.setDefault(locale)
        QApplication.removeTranslator(self.translator)
        qDebug("Changing Language to %s" % locale.name())
        if self.translator.load(locale, "ui", "_", "ui_files/translations"):
            if QApplication.installTranslator(self.translator):
                self.ui.retranslateUi(self)
                self.retranslateWatcher()
                self.retranslateOpenDialogs()
                LANGUAGE_ACTIONS[index].setChecked(True)
                for s_lang in SupportedLanguages:
                    LANGUAGE_ACTIONS[s_lang.value[0]].setText(s_lang.value[1]())
            else:
                qWarning("Could not install translator")
        else:
            qWarning("Could not load translation to %s!" % self.translator.language())

    def __add_ads_button_clicked(self):
        if self.add_ads_dialog is not None:
            return
        self.add_ads_dialog = AddADSInterface(self)
        self.add_ads_dialog.show()
        self.add_ads_dialog.ui.add_new.clicked.connect(self.__add_ads_add_new_button_clicked)
        self.add_ads_dialog.ui.buttons.clicked.connect(self.__close_add_ads_dialog)
        self.add_ads_dialog.finished.connect(self.set_ads_dialog_to_none)

    def __close_add_ads_dialog(self):
        self.add_ads_dialog.close()
        self.add_ads_dialog = None

    def set_ads_dialog_to_none(self):
        self.add_ads_dialog = None

    def __add_ads_add_new_button_clicked(self):
        radius: float
        latitude: float
        longitude: float
        try:
            radius = float(self.add_ads_dialog.ui.ads_radius.text())
            latitude = float(self.add_ads_dialog.ui.ads_latitude.text())
            longitude = float(self.add_ads_dialog.ui.ads_longitude.text())
        except:
            return
        data = AdsData()
        data.position = QGeoCoordinate(latitude, longitude)
        data.size = radius
        data.is_selected = False
        self.ui.map_view.update_ads_data(data)
        self.ui.map_view.upload_ads_data.emit()

    def __refresh_ads(self):
        if not self.server_connection.ip:
            return
        try:
            ads_list = get_ads(self.server_connection.get_address())
        except Exception as e:
            self._create_warning("Could not get HSS coordinates from server: %s" % e)
            return
        if ads_list is None:
            return
        # Elle yenileme de otomatik polling ile aynı yoldan geçiyor: haritayı,
        # tampon çemberleri, çiti ve rotayı güncelleyen tek bir akış var ve
        # kamikaze koşusu sürerken erteleme kuralı da orada uygulanıyor.
        next_seq = (self._current_snapshot.seq + 1) if self._current_snapshot is not None else 1
        self._on_hss_updated(HssSnapshot(seq=next_seq, zones=ads_list))

    def __setArmStatus(self, is_arm: int):
        self._arm_change_cooldown.start()
        qDebug("Trying to send armed status: %s" % is_arm)

        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            is_arm, # 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0
        )

    def __set_fence_clicked(self):
        if self.geofence_dialog is not None:
            return

        self.geofence_dialog = SetGeofenceInterface(self)

        if self.ui.map_view.coords_for_geofence.gc1_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc1.setText(str(self.ui.map_view.coords_for_geofence.gc1_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc1_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc2_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc2.setText(str(self.ui.map_view.coords_for_geofence.gc2_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc2_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc3_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc3.setText(str(self.ui.map_view.coords_for_geofence.gc3_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc3_v.longitude()))
        if self.ui.map_view.coords_for_geofence.gc4_v != ZERO_GEO_COORDS:
            self.geofence_dialog.ui.gc4.setText(str(self.ui.map_view.coords_for_geofence.gc4_v.latitude()) + " " + str(self.ui.map_view.coords_for_geofence.gc4_v.longitude()))
        self.geofence_dialog.show()
        self.geofence_dialog.ui.save.clicked.connect(self.__set_fence_dialog_save)
        self.geofence_dialog.finished.connect(self.__reset_geofence_dialog)

    def __set_fence_dialog_save(self):
        gc1 = self.geofence_dialog.ui.gc1.text().split()
        gc2 = self.geofence_dialog.ui.gc2.text().split()
        gc3 = self.geofence_dialog.ui.gc3.text().split()
        gc4 = self.geofence_dialog.ui.gc4.text().split()

        if len(gc1) != 2 or len(gc2) != 2 or len(gc3) != 2 or len(gc4) != 2:
            return
        self.ui.map_view.coords_for_geofence.gc1_v = QGeoCoordinate(float(gc1[0]), float(gc1[1]))
        self.ui.map_view.coords_for_geofence.gc2_v = QGeoCoordinate(float(gc2[0]), float(gc2[1]))
        self.ui.map_view.coords_for_geofence.gc3_v = QGeoCoordinate(float(gc3[0]), float(gc3[1]))
        self.ui.map_view.coords_for_geofence.gc4_v = QGeoCoordinate(float(gc4[0]), float(gc4[1]))
        self.ui.map_view.coords_for_geofence.gc_changed.emit()
        self.ui.map_view.coords_for_geofence.upload_geofence_data.emit()
        qDebug("New geofence coords: %s, %s, %s, %s" % (self.ui.map_view.coords_for_geofence.gc1_v, self.ui.map_view.coords_for_geofence.gc2_v, self.ui.map_view.coords_for_geofence.gc3_v, self.ui.map_view.coords_for_geofence.gc4_v))
        self.geofence_dialog.close()

    def _enable_fence(self) -> None:
        if self.mavlink_connection is None:
            return
        if self.current_pilot != MAV_AUTOPILOT_PX4:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                MAV_CMD_DO_FENCE_ENABLE,
                0,
                1,
                0,0,0,0,0,0
            )

    requested_to_send_fence_with_fence: bool = False

    fence_upload_in_progress: bool = False           # FIX: race condition kilidi
    pending_fence_ads_list: list = None              # FIX: yükleme sırasında gelen yeni istek kuyruğu
    def send_fence_mission_data(self, index: int, use_item_int: bool):
        ads_list_len = len(self.ads_list_cache)
        if ads_list_len == 0 and not self.requested_to_send_fence_with_fence:
            return
        qDebug("Requested Mission Data at index %s" % index)
        if index >= ads_list_len:
            coords = [self.ui.map_view.coords_for_geofence.gc1_v, self.ui.map_view.coords_for_geofence.gc2_v,
                      self.ui.map_view.coords_for_geofence.gc3_v, self.ui.map_view.coords_for_geofence.gc4_v]

            qDebug("Sending fence with position %s" % coords[index - ads_list_len])

            if use_item_int:
                self.mavlink_connection.mav.mission_item_int_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL_INT,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    0,
                    0,
                    4,
                    0,
                    0,
                    0,
                    int(coords[index - ads_list_len].latitude() * 1e7),
                    int(coords[index - ads_list_len].longitude() * 1e7),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
            else:
                self.mavlink_connection.mav.mission_item_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL,
                    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                    0,
                    0,
                    4,
                    0,
                    0,
                    0,
                    coords[index - ads_list_len].latitude(),
                    coords[index - ads_list_len].longitude(),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
        else:
            qDebug("Sending ads with position %s" % self.ads_list_cache[index].position)
            if use_item_int:
                self.mavlink_connection.mav.mission_item_int_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL_INT,
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    0,
                    0,
                    self.ads_list_cache[index].size,
                    0,
                    0,
                    0,
                    int(self.ads_list_cache[index].position.latitude() * 1e7),
                    int(self.ads_list_cache[index].position.longitude() * 1e7),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
            else:
                self.mavlink_connection.mav.mission_item_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    index,
                    MAV_FRAME_GLOBAL,
                    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                    0,
                    0,
                    self.ads_list_cache[index].size,
                    0,
                    0,
                    0,
                    self.ads_list_cache[index].position.latitude(),
                    self.ads_list_cache[index].position.longitude(),
                    0,
                    MAV_MISSION_TYPE_FENCE
                )
        self.fence_upload_timout.start()
        size: int = ads_list_len + 4 if self.requested_to_send_fence_with_fence else ads_list_len
        if size == index + 1:
            self.fence_upload_timout.stop()
            self.ads_list_cache = []
            self.requested_to_send_fence_with_fence = False
            self.fence_upload_in_progress = False   # FIX: kilit serbest
            self._fence_upload_retries = 0
            # Çit ancak tamamı araca yüklendikten SONRA aktif edilir. Eskiden
            # yükleme başlatılır başlatılmaz aktif ediliyordu.
            self._enable_fence()
            qDebug("[Fence] Çit yüklendi ve aktif edildi")
            # FIX: yükleme biterken bekleyen istek varsa hemen işle
            if self.pending_fence_ads_list is not None:
                pending = self.pending_fence_ads_list
                self.pending_fence_ads_list = None
                self.update_geofence_data(pending)

    ads_list_cache: list[AdsData] = []

    def update_geofence_data(self, ads_list: list[AdsData]):
        if self.mavlink_connection is None:
            qWarning("Tried to sent geofence data when there is no mavlink connection")
            return
        # FIX: Fence yükleme devam ederken yeni istek gelirse kuyruğa al (race condition önleme)
        if self.fence_upload_in_progress:
            qDebug("Fence upload in progress, queuing new request")
            self.pending_fence_ads_list = ads_list
            return
        has_a_fence = self.ui.map_view.coords_for_geofence.is_set
        self.mavlink_connection.mav.mission_clear_all_send(self.mavlink_connection.target_system, self.mavlink_connection.target_component, MAV_MISSION_TYPE_FENCE)
        fence_count: int = len(ads_list)
        if has_a_fence:
            fence_count += 4
            self.requested_to_send_fence_with_fence = True
        if fence_count == 0:
            qDebug("Fence count is 0, not sending anything for fence")
            return
        qDebug("Sending mission count: %s" % fence_count)
        self.ads_list_cache = ads_list
        self.fence_upload_in_progress = True         # FIX: kilidi aç
        self.mavlink_connection.mav.mission_count_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            fence_count,
            MAV_MISSION_TYPE_FENCE
        )
        self.fence_upload_timout.start()

    def fence_upload_reset(self):
        self._create_warning("Fence Upload taking really long, probably vehicle connection has been lost")
        self.ads_list_cache = []
        self.requested_to_send_fence_with_fence = False
        self.fence_upload_in_progress = False        # FIX: timeout'da da kilidi serbest bırak
        # Bekleyen istek ARTIK atılmıyor: eskiden burada silindiği için, ilk
        # (dar yarıçaplı) yükleme timeout'a düşünce tamponlu çit hiç yüklenmiyor
        # ve araçta yanlış çit kalıyordu. Sınırlı sayıda tekrar denenir.
        pending = self.pending_fence_ads_list
        self.pending_fence_ads_list = None
        if pending is None:
            return
        if self._fence_upload_retries >= 2:
            self._fence_upload_retries = 0
            self._create_warning("Çit yüklenemedi — araçtaki çit güncel DEĞİL!")
            return
        self._fence_upload_retries += 1
        qWarning("[Fence] Yükleme zaman aşımı, tekrar deneniyor (%d/2)" % self._fence_upload_retries)
        self.update_geofence_data(pending)

    def __reset_geofence_dialog(self):
        self.geofence_dialog = None

    kamikaze_start: GpsSaati
    kamikaze_state: KamikazeState
    kamikaze_original_alt: float
    kamikaze_target_lat: float
    kamikaze_target_lon: float
    kamikaze_previous_mode: int | None
    kamikaze_timer: QTimer
    kamikaze_recover_heading: float
    kamikaze_alt_history: list[tuple[float, float]]  # (zaman, irtifa)
    kamikaze_dive_sink_max: float
    kamikaze_lowest_alt: float
    waits_for_qr: bool
    kamikaze_qr_text: str

    # --- Parametre yazma: sahiplik + teyit ---------------------------------

    def _kamikaze_owns_vehicle(self) -> bool:
        return self.kamikaze_state != KamikazeState.IDLE

    def __send_param_writes(self, writes: list[tuple[bytes, float]]) -> None:
        try:
            for name, value in writes:
                self.mavlink_connection.mav.param_set_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    name,
                    value,
                    9
                )
        except OSError as e:
            # Bağlantı az önce ölmüş olabilir (USB çıktı, seri port düştü).
            # Baseline yazıları tam da o anda -- kamikaze bırakılırken --
            # gönderiliyor, o yüzden burada patlamak sinyal zincirini kırardı.
            qWarning("Parametre gönderilemedi, bağlantı düşmüş olabilir: %s" % e)

    def __set_param(self, name: bytes, value: float,
                    owner: ParamOwner = ParamOwner.BASELINE, verify: bool = True) -> None:
        self.__set_param_group(name.decode('ascii'), [(name, value)], owner, verify)

    def __set_param_group(self, key: str, writes: list[tuple[bytes, float]],
                          owner: ParamOwner = ParamOwner.BASELINE, verify: bool = True) -> None:
        """
        Tek bir mantıksal parametreyi yazar. writes birden fazla mavlink ismi
        içerebilir (eski/yeni firmware alias'ları); teyit için HERHANGİ birinden
        PARAM_VALUE gelmesi yeterlidir, çünkü firmware'de olmayan alias hiç
        cevap vermez.

        owner, o an aracı kimin sürdüğüdür. Kamikaze koşusu sürerken HSS
        katmanından (veya başka bir yerden) gelen bir yazı sessizce uygulanmaz,
        engellenir ve loglanır -- iki katmanın aynı anda parametre yazmasının
        önündeki tek gerçek engel bu.
        """
        if self.mavlink_connection is None:
            return
        active_owner = ParamOwner.KAMIKAZE if self._kamikaze_owns_vehicle() else ParamOwner.BASELINE
        if owner is not active_owner:
            qWarning("Parametre yazısı engellendi (%s): sahip %s, isteyen %s"
                     % (key, active_owner.name, owner.name))
            return
        self.__send_param_writes(writes)
        if not verify:
            self._param_pending.pop(key, None)
            return
        # owner da saklanıyor: tekrar gönderim sırasında aracın sahibi
        # değişmişse bu yazı bayattır ve gönderilmemeli.
        self._param_pending[key] = {'writes': writes, 'attempts': 1, 'owner': owner}
        if not self._param_retry_timer.isActive():
            self._param_retry_timer.start()

    def _on_param_value(self, name: str, value: float) -> None:
        for key in list(self._param_pending.keys()):
            entry = self._param_pending[key]
            for param_name, expected in entry['writes']:
                if param_name.decode('ascii', 'replace') != name:
                    continue
                if abs(value - expected) <= max(1e-3, abs(expected) * 1e-4):
                    del self._param_pending[key]
                break
        if not self._param_pending:
            self._param_retry_timer.stop()

    def __retry_pending_params(self) -> None:
        if self.mavlink_connection is None:
            self._param_pending.clear()
            self._param_retry_timer.stop()
            return
        active_owner = ParamOwner.KAMIKAZE if self._kamikaze_owns_vehicle() else ParamOwner.BASELINE
        for key in list(self._param_pending.keys()):
            entry = self._param_pending[key]
            # Sahiplik kontrolü tekrar gönderimde de geçerli. Bu olmadan,
            # kamikaze başlamadan hemen önce kuyruğa girmiş bir baseline yazısı
            # (ör. TECS_SINK_MAX=5) dalışın ortasında yeniden gönderilip
            # kamikaze'nin değerini eziyordu -- __set_param_group'taki kapı
            # buradan atlanıyordu.
            if entry['owner'] is not active_owner:
                qDebug("Bayat parametre yazısı düşürüldü: %s (%s -> %s)"
                       % (key, entry['owner'].name, active_owner.name))
                del self._param_pending[key]
                continue
            if entry['attempts'] >= PARAM_MAX_ATTEMPTS:
                del self._param_pending[key]
                self._create_warning("Parametre yazılamadı: %s — otopilot teyit vermedi!" % key)
                continue
            entry['attempts'] += 1
            qDebug("Parametre tekrar gönderiliyor: %s (deneme %d)" % (key, entry['attempts']))
            self.__send_param_writes(entry['writes'])
        if not self._param_pending:
            self._param_retry_timer.stop()

    def apply_baseline_params(self) -> None:
        """
        Aracı HSS'in planladığı normal seyir durumuna döndürür.

        Kamikaze bitişi, force-end, reposition bırakma ve bağlantı kurulması --
        hepsi buradan geçer. Baseline'ın tek tanımı BASELINE_PARAMS tablosudur;
        buraya bir literal yazmak, bu düzeltmenin çözdüğü hatayı geri getirir.
        """
        if self.mavlink_connection is None:
            return
        for key, writes in BASELINE_PARAMS:
            self.__set_param_group(key, writes, ParamOwner.BASELINE)
        self.ui.map_view.vehicle_locked = False
        qDebug("[Params] Baseline uygulandı (%d parametre)" % len(BASELINE_PARAMS))

    def apply_hss_safety_params(self) -> None:
        """Çit/rally/AUTO seyrüsefer parametreleri — bağlantıda bir kez."""
        if self.mavlink_connection is None:
            return
        for key, writes in HSS_SAFETY_PARAMS:
            self.__set_param_group(key, writes, ParamOwner.BASELINE)
        qDebug("[Params] HSS güvenlik parametreleri uygulandı (%d parametre)" % len(HSS_SAFETY_PARAMS))

    def __start_kamikaze(self):
        if self.kamikaze_state != KamikazeState.IDLE:
            qDebug("Cancelling Kamikaze")
            self.__finish_kamikaze()
            return

        if self.mavlink_connection is None:
            qWarning("No UAV Connection, Can not Start Kamikaze")
            return
        try:
            latitude: float = float(self.ui.kamikaze_latitude.text())
            longitude: float = float(self.ui.kamikaze_longitude.text())
        except ValueError:
            qWarning("Invalid Kamikaze Coordinates")
            return

        self.kamikaze_target_lat = latitude
        self.kamikaze_target_lon = longitude

        self.next_telemetry.lock.lockForRead()
        self.kamikaze_start = self.next_telemetry.gps_saati
        self.kamikaze_original_alt = self.next_telemetry.iha_irtifa
        current_lat: float = self.next_telemetry.iha_enlem
        current_lon: float = self.next_telemetry.iha_boylam
        self.next_telemetry.lock.unlock()
        if self.kamikaze_original_alt <= 0:
            qWarning("No Valid UAV Info Found, Cancelling Kamikaze")
            return
        if current_lat == 0 and current_lon == 0:
            self._create_warning("No valid UAV position yet, refusing to start kamikaze")
            return
        if not self.uav_is_armed:
            self._create_warning("UAV is not armed, refusing to start kamikaze")
            return
        if self.kamikaze_original_alt < 80.0:
            self._create_warning("Altitude %.1fm is below the 80m kamikaze minimum, refusing to start" % self.kamikaze_original_alt)
            return
        # Giriş kapısı. Koşu boyunca HSS katmanı donduruluyor (fence yüklemesi,
        # rota planlama, mission yüklemesi yok), o yüzden koşuya girerken
        # durumun temiz olduğundan emin olmak gerekiyor -- yoksa "gözünü kapat
        # ve dal" olur.
        if self.fence_upload_in_progress or (self.mavlink_worker is not None
                                             and self.mavlink_worker._mission_upload_state > 0):
            self._create_warning("Çit/rota yüklemesi sürüyor, kamikaze başlatılmıyor")
            return
        zones = self._hss_zones()
        hit = point_hits_zone(latitude, longitude, zones)
        if hit is not None:
            self._create_warning("Kamikaze hedefi HSS #%s bölgesinin içinde — başlatılmıyor" % hit.id)
            return
        hit = segment_hits_zone(current_lat, current_lon, latitude, longitude, zones)
        if hit is not None:
            self._create_warning("DİKKAT: Hedefe giriş hattı HSS #%s bölgesini kesiyor!" % hit.id)

        self.kamikaze_previous_mode = self.ui.fly_mode_combobox.currentIndex()
        self.kamikaze_state = KamikazeState.APPROACHING
        self._kamikaze_run_started_at = time.monotonic()
        # Araç artık kamikaze'nin: HSS katmanı ve harita reposition'ı bu andan
        # itibaren parametre yazamaz (bkz. __set_param_group, MapWidget).
        self.ui.map_view.vehicle_locked = True
        self.kamikaze_timer.start()

        # Dive pitch limit and run-in roll limit. Set both the old (centidegree:
        # LIM_*) and new (degree: *_DEG) ArduPlane parameter names so this works
        # regardless of firmware version (4.1+ renamed these params).
        # TECS_PITCH_MIN is the limit TECS itself uses in autothrottle modes and
        # it overrides LIM_PITCH_MIN unless it is left at 0, so it has to be set
        # too or GUIDED will never let the nose past its cruise limit.
        # THR_MAX is set explicitly for the run-in because a still-active
        # reposition or an earlier phase may have left it somewhere else.
        dive_pitch_min: float = -(KAMIKAZE_DIVE_ANGLE + KAMIKAZE_DIVE_PITCH_MARGIN)
        k = ParamOwner.KAMIKAZE
        self.__set_param(b'THR_MAX', KAMIKAZE_APPROACH_THR_MAX, k)
        self.__set_param_group('PTCH_LIM_MIN_DEG',
                               [(b'PTCH_LIM_MIN_DEG', dive_pitch_min),
                                (b'LIM_PITCH_MIN', dive_pitch_min * 100.0)], k)
        # TECS applies its own floor on top of LIM_PITCH_MIN and the tighter of
        # the two wins, so both have to be opened or the dive never gets past
        # the cruise limit.
        self.__set_param(b'TECS_PITCH_MIN', dive_pitch_min, k)
        self.__set_param_group('ROLL_LIMIT_DEG',
                               [(b'ROLL_LIMIT_DEG', KAMIKAZE_APPROACH_ROLL_LIMIT),
                                (b'LIM_ROLL_CD', KAMIKAZE_APPROACH_ROLL_LIMIT * 100.0)], k)
        self.__set_param(b'TECS_CLMB_MAX', KAMIKAZE_TECS_CLMB_MAX, k)
        # Eski ve yeni isim birlikte -- bkz. BASELINE_PARAMS'taki not. Bu yazı
        # düştüğünde TECS hızı seyir limitinde tutuyor ve dalış hiç gelişmiyor.
        self.__set_param_group('ARSPD_FBW_MAX',
                               [(b'AIRSPEED_MAX', KAMIKAZE_ARSPD_FBW_MAX),
                                (b'ARSPD_FBW_MAX', KAMIKAZE_ARSPD_FBW_MAX)], k)
        # Eski ve yeni isim birlikte -- bkz. BASELINE_PARAMS'taki not.
        self.__set_param_group('ALT_SLOPE_MIN',
                               [(b'ALT_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN),
                                (b'GLIDE_SLOPE_MIN', KAMIKAZE_GLIDE_SLOPE_MIN)], k)

        self.mavlink_connection.set_mode_apm(PLANE_MODE_GUIDED)
        self._mode_change_cooldown.start()
        self.kamikaze_alt_history.clear()
        self.kamikaze_dive_sink_max = 0.0
        aim_lat, aim_lon = self.__kamikaze_aim_point(current_lat, current_lon)
        self.__send_guided_target(aim_lat, aim_lon, KAMIKAZE_APPROACH_ALT)
        self._kamikaze_target_cooldown.start()
        self.waits_for_qr = True
        self.kamikaze_qr_text = ""
        qDebug("Kamikaze Started")

    def __send_guided_target(self, latitude: float, longitude: float, altitude: float):
        # Points the GUIDED destination at a coordinate. The command carries
        # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE so it is accepted (and switches to
        # GUIDED) even if the mode change was missed. Sent once per phase: see
        # KAMIKAZE_TARGET_REFRESH for why it must not be repeated mid-dive.
        if self.mavlink_connection is None:
            return
        self.mavlink_connection.mav.command_int_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_CMD_DO_REPOSITION,
            0,
            0,
            -1.0,  # ground speed, -1 keeps the current one
            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            KAMIKAZE_LOITER_RADIUS,
            float('nan'),  # yaw, NaN keeps the current one
            int(latitude * 10 ** 7),
            int(longitude * 10 ** 7),
            max(KAMIKAZE_MIN_AIM_ALT, altitude)
        )

    def __kamikaze_loop(self):
        if self.kamikaze_state == KamikazeState.IDLE:
            return

        # Watchdog: koşunun kendiliğinden bitmediği durumlar var (irtifa
        # yetmezse APPROACHING sonsuza kadar tur atar) ve koşu sürerken HSS
        # katmanı donuk. Bkz. KAMIKAZE_MAX_RUN_TIME.
        if time.monotonic() - self._kamikaze_run_started_at > KAMIKAZE_MAX_RUN_TIME:
            self._create_warning("Kamikaze %.0f saniyede tamamlanamadı, iptal ediliyor"
                                 % KAMIKAZE_MAX_RUN_TIME)
            self.__finish_kamikaze()
            return

        self.next_telemetry.lock.lockForRead()
        current_lat: float = self.next_telemetry.iha_enlem
        current_lon: float = self.next_telemetry.iha_boylam
        current_alt: float = self.next_telemetry.iha_irtifa
        ground_speed: float = self.next_telemetry.iha_hiz
        pitch: float = self.next_telemetry.iha_dikilme
        self.next_telemetry.lock.unlock()

        if current_lat == 0 and current_lon == 0:
            return

        distance: float = MainWindow.__calculate_distance(current_lat, current_lon, self.kamikaze_target_lat, self.kamikaze_target_lon)
        sink_rate: float = self.__update_sink_rate(current_alt)

        if self.kamikaze_state == KamikazeState.APPROACHING:
            # GUIDED flies the run-in and the climb by itself; the destination
            # is only repeated slowly so a dropped command still gets through,
            # and it is recomputed each time because the line it has to sit on
            # is the one the vehicle is currently running in along.
            if not self._kamikaze_target_cooldown.isActive():
                aim_lat, aim_lon = self.__kamikaze_aim_point(current_lat, current_lon)
                self.__send_guided_target(aim_lat, aim_lon, KAMIKAZE_APPROACH_ALT)
                self._kamikaze_target_cooldown.start()
            # Dive from wherever the QR point is on the nose plus the rotation
            # lead, but never closer than the configured minimum.
            dive_at: float = max(KAMIKAZE_DIVE_START_DISTANCE,
                                 current_alt / math.tan(math.radians(KAMIKAZE_DIVE_ANGLE)) + KAMIKAZE_DIVE_ROTATION_LEAD)
            if distance <= dive_at:
                if current_alt < KAMIKAZE_APPROACH_ALT - KAMIKAZE_APPROACH_ALT_TOLERANCE:
                    if not self._kamikaze_climb_warned:
                        self._kamikaze_climb_warned = True
                        self._create_warning("Only at %.0fm of the %.0fm dive altitude, going around"
                                             % (current_alt, KAMIKAZE_APPROACH_ALT))
                else:
                    qDebug(f"Remaining Distance is {distance:.1f}m of {dive_at:.1f}m, entering dive state")
                    self.__enter_dive(current_lat, current_lon, current_alt, distance, ground_speed, pitch)

        elif self.kamikaze_state == KamikazeState.DIVING:
            # The destination is left alone here on purpose; the angle is held
            # by the sink rate instead, which is a parameter write and does not
            # disturb navigation. Break off early enough that the pull-out
            # bottoms out at MIN_ALT instead of starting there.
            self.__update_dive_sink_rate(ground_speed, pitch)
            # Dalışın her tick'i loglanıyor. Tek bir "şu kadar derece daldı"
            # sayısıyla açının neden sığ kaldığını ayırt etmek mümkün değil:
            # tavan mı bağlıyor, burun mu dönemiyor, dalış mı erken bitiyor --
            # üçü de aynı sonucu veriyor ama çözümleri farklı.
            data_age: float = self.__altitude_data_age()
            # Uçuş yolu açısı doğrudan pitch'ten: hücum açısı ölçülerek ~0
            # bulundu ve pitch beş kat hızlı, gürültüsüz bir sinyal.
            # Bkz. __sink_from_pitch. sink_rate (irtifa türevi) log'da çapraz
            # kontrol olarak kalıyor.
            sink_est: float = MainWindow.__sink_from_pitch(ground_speed, pitch)
            flown_angle: float = max(0.0, -pitch)
            # Hücum açısı = pitch + uçuş yolu açısı (pitch burun aşağı negatif,
            # flown_angle alçalırken pozitif). 0'a yakınsa burun uçuş yoluyla
            # hizalı, yani gerçek bir dalış. Büyükse uçak burnu yukarıda
            # "çökerek" alçalıyor demektir -- bu hem sürüklemeyi patlatıp hızın
            # artmasını engeller, hem de KAMERAYI hedeften başka yere baktırır.
            aoa: float = pitch + math.degrees(math.atan2(sink_rate, max(ground_speed, 0.1)))
            qDebug("[Dive] t=%.1fs alt=%.1f gs=%.1f sink=%.1f(olc %.1f) aci=%.1f hucum=%.1f talep=%.1f yas=%.2fs"
                   % (time.monotonic() - self._kamikaze_run_started_at, current_alt,
                      ground_speed, sink_est, sink_rate, flown_angle, aoa,
                      self.kamikaze_dive_sink_max, data_age))
            self.kamikaze_steepest_angle = max(self.kamikaze_steepest_angle, flown_angle)
            self.kamikaze_steepest_pitch = min(self.kamikaze_steepest_pitch, pitch)
            pullout_loss: float = MainWindow.__pullout_altitude_loss(
                self.__recent_peak_sink(sink_est), data_age)
            if current_alt - pullout_loss <= KAMIKAZE_MIN_ALT:
                qDebug("Sink rate %.1fm/s at %.1fm (%.2fs eski) needs %.1fm to pull out, entering recovering state"
                       % (sink_rate, current_alt, data_age, pullout_loss))
                self.kamikaze_dive_duration = time.monotonic() - self.kamikaze_dive_started_at
                self.__enter_recovery(current_lat, current_lon)

        elif self.kamikaze_state == KamikazeState.RECOVERING:
            # The vehicle keeps sinking into the pull-out; this is the number
            # KAMIKAZE_PULLOUT_TIME is meant to land on MIN_ALT.
            self.kamikaze_lowest_alt = min(self.kamikaze_lowest_alt, current_alt)
            if current_alt >= KAMIKAZE_RECOVER_ALT:
                qDebug("Recovering complete, returning to last mode")
                self.kamikaze_state = KamikazeState.RESUMING
                self.__finish_kamikaze()

    @staticmethod
    def __pullout_altitude_loss(sink_rate: float, data_age: float) -> float:
        # How much further the vehicle sinks between the recovery being
        # commanded and it stopping going down (KAMIKAZE_PULLOUT_TIME), PLUS how
        # far it has already sunk since the altitude being judged was measured.
        # The altitude in hand is a telemetry sample that can be half a second
        # old; ignoring that age is why a run aimed at a 70 m floor bottomed out
        # at 61 m.
        return sink_rate * (KAMIKAZE_PULLOUT_TIME + data_age)

    def __update_sink_rate(self, current_alt: float) -> float:
        # Only distinct altitudes are recorded, with the time they were seen, and
        # the rate is taken over the real span between the oldest and newest
        # sample. See KAMIKAZE_SINK_WINDOW for why the old fixed-interval
        # assumption produced a sawtooth instead of a sink rate.
        now: float = time.monotonic()
        if not self.kamikaze_alt_history or self.kamikaze_alt_history[-1][1] != current_alt:
            self.kamikaze_alt_history.append((now, current_alt))
        # Keep at least two samples so a rate is always available, and drop
        # anything older than the window so the estimate does not lag the dive.
        while len(self.kamikaze_alt_history) > 2 and \
                now - self.kamikaze_alt_history[0][0] > KAMIKAZE_SINK_WINDOW:
            self.kamikaze_alt_history.pop(0)
        if len(self.kamikaze_alt_history) < 2:
            return 0.0
        span: float = self.kamikaze_alt_history[-1][0] - self.kamikaze_alt_history[0][0]
        if span < 1e-3:
            return 0.0
        return max(0.0, (self.kamikaze_alt_history[0][1] - self.kamikaze_alt_history[-1][1]) / span)

    def __recent_peak_sink(self, sink_rate: float) -> float:
        """
        Son KAMIKAZE_PULLOUT_PEAK_WINDOW saniyedeki en yüksek alçalma hızı.

        Pull-out tahmini anlık ölçümle yapılamaz: ölçülen alçalma hızı yarım
        saniyede bir 11 ile 15 m/s arasında oynuyor ve tetik tesadüfen düşük bir
        okumaya denk gelirse kayıp olduğundan az tahmin edilip uçak tabanın
        altına iniyor (ölçüm: 12.1 okundu, gerçek kayıp 15 m/s'ye karşılık
        gelen kadardı). Tepe tutma her zaman erken tarafta hata yapar.
        """
        now: float = time.monotonic()
        self.kamikaze_sink_peaks.append((now, sink_rate))
        while len(self.kamikaze_sink_peaks) > 1 and \
                now - self.kamikaze_sink_peaks[0][0] > KAMIKAZE_PULLOUT_PEAK_WINDOW:
            self.kamikaze_sink_peaks.pop(0)
        return max(s for _, s in self.kamikaze_sink_peaks)

    def __altitude_data_age(self) -> float:
        """Elimizdeki irtifa örneği kaç saniye önce ölçüldü."""
        if not self.kamikaze_alt_history:
            return 0.0
        return max(0.0, time.monotonic() - self.kamikaze_alt_history[-1][0])

    def __kamikaze_aim_point(self, current_lat: float, current_lon: float) -> tuple[float, float]:
        # The QR point pushed KAMIKAZE_AIM_OVERSHOOT further along the line the
        # vehicle is running in on, so the straight line to the destination
        # still passes exactly over the QR code while staying far enough away
        # for L1 to keep flying it straight. See KAMIKAZE_AIM_OVERSHOOT.
        bearing: float = math.degrees(MainWindow.__bearing_to(current_lat, current_lon, self.kamikaze_target_lat, self.kamikaze_target_lon))
        return MainWindow.__offset_coords(self.kamikaze_target_lat, self.kamikaze_target_lon, bearing, KAMIKAZE_AIM_OVERSHOOT)

    @staticmethod
    def __sink_from_pitch(ground_speed: float, pitch: float) -> float:
        """
        Alçalma hızını irtifa türevi yerine pitch'ten kestirir.

        Ölçüldü (2026-08-14): dalış boyunca hücum açısı -8 ile +0.2 derece
        arasında, ortalama ~-3 -- yani burun uçuş yoluyla hizalı ve pitch ZATEN
        uçuş yolu açısı. Pitch ATTITUDE mesajıyla ~10 Hz ve pürüzsüz geliyor;
        irtifa ise ~2 Hz ve kuantalı, türevi ±3 derecelik testere dişi üretiyor.
        gs*tan(pitch) ölçülen alçalma hızını birebir tutturuyor (18.6*0.80=14.9,
        ölçüm 14.9) ama gürültüsüz.

        Bu, hem trim döngüsünün gürültü kovalamasını, hem de pull-out tahmininin
        salınımın dip noktasına denk gelmesini bitiriyor.
        """
        if pitch >= 0.0:
            return 0.0
        return max(0.0, ground_speed * math.tan(math.radians(min(-pitch, 85.0))))

    def __update_dive_sink_rate(self, ground_speed: float, pitch: float):
        # Sink rate and ground speed are the two legs of the dive angle:
        # sink = ground_speed * tan(angle). TECS_SINK_MAX caps the descent TECS
        # is willing to demand, so slaving it to the current ground speed holds
        # the angle as the dive accelerates. A fixed cap cannot: it gives
        # asin(cap/V), which flattens out as speed builds.
        if ground_speed < KAMIKAZE_MIN_VALID_SPEED:
            return
        # Ask for the angle plus whatever the vehicle is currently flying short
        # of it, so TECS's own lag lands on DIVE_ANGLE instead of under it.
        #
        # Uçulan açı pitch'ten alınıyor, irtifa türevinden değil: hücum açısı
        # ölçülerek ~0 bulundu, yani pitch zaten uçuş yolu açısı ve çok daha
        # temiz. Eskiden bu değer ±3 derece zıpladığı için talep de 24 ile 31
        # m/s arasında salınıyordu; TECS'in 3 saniyelik zaman sabiti yarım
        # saniyede bir değişen bir talebi zaten takip edemez.
        flown: float = max(0.0, -pitch)
        trim: float = KAMIKAZE_DIVE_TRIM_GAIN * (KAMIKAZE_DIVE_ANGLE - flown)
        commanded: float = KAMIKAZE_DIVE_ANGLE + clamp(trim, 0.0, KAMIKAZE_DIVE_TRIM_MAX)
        wanted: float = min(KAMIKAZE_TECS_SINK_MAX, ground_speed * math.tan(math.radians(commanded)))
        if abs(wanted - self.kamikaze_dive_sink_max) < KAMIKAZE_SINK_STEP:
            return
        self.kamikaze_dive_sink_max = wanted
        # verify=True olmak ZORUNDA. Bu tek yazı dalışın tamamını belirliyor ve
        # kamikaze_dive_sink_max yukarıdaki eşik kontrolünde "araçta bu değer
        # var" varsayımı olarak kullanılıyor: paket düşerse kod aracın 26 m/s
        # tavanda olduğunu sanır, araç ise baseline'daki 5 m/s'de kalır ve
        # hesaplanan değer 0.5'ten fazla oynamadığı için bir daha ASLA
        # gönderilmez. 5 m/s tavan ~12 m/s yer hızında 22 derecelik bir dalış
        # demek. Teyit kuyruğu parametre adına göre tutulduğu için burada
        # birikme olmaz: yeni yazı eskisinin yerine geçer.
        self.__set_param(b'TECS_SINK_MAX', wanted, ParamOwner.KAMIKAZE)

    def __enter_dive(self, current_lat: float, current_lon: float, current_alt: float, distance: float, ground_speed: float, pitch: float):
        self.next_telemetry.lock.lockForRead()
        yaw: float = self.next_telemetry.iha_yonelme
        self.next_telemetry.lock.unlock()
        bearing: float = math.degrees(MainWindow.__bearing_to(current_lat, current_lon, self.kamikaze_target_lat, self.kamikaze_target_lon))
        heading_error: float = abs((bearing - yaw + 180.0) % 360.0 - 180.0)
        if heading_error > KAMIKAZE_MAX_DIVE_HEADING_ERROR:
            self._create_warning("Dive starts %.0f degrees off the target bearing, QR may not be readable" % heading_error)
        # Engine off, wings pinned, and pitch handed entirely to the altitude
        # demand so TECS stops trading the descent against airspeed.
        k = ParamOwner.KAMIKAZE
        self.__set_param(b'THR_MAX', KAMIKAZE_DIVE_THR_MAX, k)
        self.__set_param(b'TECS_SPDWEIGHT', KAMIKAZE_TECS_SPDWEIGHT, k)
        self.__set_param(b'TECS_TIME_CONST', KAMIKAZE_TECS_TIME_CONST, k)
        self.__set_param(b'TECS_VERT_ACC', KAMIKAZE_TECS_VERT_ACC, k)
        self.__set_param_group('ROLL_LIMIT_DEG',
                               [(b'ROLL_LIMIT_DEG', KAMIKAZE_DIVE_ROLL_LIMIT),
                                (b'LIM_ROLL_CD', KAMIKAZE_DIVE_ROLL_LIMIT * 100.0)], k)
        self.kamikaze_dive_sink_max = 0.0
        self.kamikaze_lowest_alt = current_alt
        self.kamikaze_steepest_angle = 0.0
        self.kamikaze_steepest_pitch = 0.0
        self.kamikaze_sink_peaks = []
        self.kamikaze_dive_started_at = time.monotonic()
        self.kamikaze_dive_start_alt = current_alt
        self.kamikaze_dive_duration = 0.0
        self.__update_dive_sink_rate(ground_speed, pitch)
        # One destination for the whole dive: down the same line, past the QR
        # point, at ground level. With the glide slope off the vehicle stops
        # rationing that altitude over the distance and just descends, which is
        # what the sink rate above is there to shape.
        aim_lat, aim_lon = self.__kamikaze_aim_point(current_lat, current_lon)
        self.__send_guided_target(aim_lat, aim_lon, KAMIKAZE_MIN_AIM_ALT)
        qDebug("Diving from %.1fm at %.1fm out, %.1fm/s ground speed" % (current_alt, distance, ground_speed))
        self.kamikaze_state = KamikazeState.DIVING

    def __enter_recovery(self, current_lat: float, current_lon: float):
        self.next_telemetry.lock.lockForRead()
        self.kamikaze_recover_heading = self.next_telemetry.iha_yonelme
        self.next_telemetry.lock.unlock()
        k = ParamOwner.KAMIKAZE
        self.__set_param(b'THR_MAX', KAMIKAZE_RECOVER_THR_MAX, k)
        self.__set_param(b'TECS_SPDWEIGHT', CRUISE_TECS_SPDWEIGHT, k)
        self.__set_param_group('PTCH_LIM_MIN_DEG',
                               [(b'PTCH_LIM_MIN_DEG', KAMIKAZE_RECOVER_PITCH_MIN),
                                (b'LIM_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN * 100.0)], k)
        self.__set_param(b'TECS_PITCH_MIN', KAMIKAZE_RECOVER_PITCH_MIN, k)
        # Give some bank authority back: the dive pinned it at 15 deg so the
        # nose would hold the QR line, but the recovery may have to steer away
        # from an HSS zone (see __kamikaze_recover_lead) and cannot do that at
        # 15. Not all the way back to the run-in value -- see
        # KAMIKAZE_RECOVER_ROLL_LIMIT.
        self.__set_param_group('ROLL_LIMIT_DEG',
                               [(b'ROLL_LIMIT_DEG', KAMIKAZE_RECOVER_ROLL_LIMIT),
                                (b'LIM_ROLL_CD', KAMIKAZE_RECOVER_ROLL_LIMIT * 100.0)], k)
        # One destination again: ahead of the vehicle, high enough that it keeps
        # climbing through RECOVER_ALT. Being below the destination is the case
        # where ArduPlane climbs at its max rate instead of following a slope,
        # which is what pulls the nose up.
        lead_lat, lead_lon = self.__kamikaze_recover_lead(current_lat, current_lon)
        self.__send_guided_target(lead_lat, lead_lon, KAMIKAZE_APPROACH_ALT)
        self.kamikaze_state = KamikazeState.RECOVERING

    def __kamikaze_recover_lead(self, current_lat: float, current_lon: float) -> tuple[float, float]:
        """
        Kurtarma tırmanışının hedef noktası. Tercih edilen nokta dalış yönünde
        düz ileridedir, ama uçak dalışın dibinde bir HSS bölgesinin kenarında
        olabilir ve düz ileri devam etmek onu bölgenin içine sokabilir. Rota
        planlayıcı sadece AUTO görevini düzeltiyor, GUIDED'ın bu bacağını değil
        -- o yüzden kontrol burada yapılıyor.

        Önce yön sapması, sonra kısaltma denenir; hiçbiri temiz değilse düz
        ileri kullanılır ve operatör uyarılır (tırmanmamak her hâlükârda daha
        kötü).
        """
        zones = self._hss_zones()
        heading = self.kamikaze_recover_heading
        if not zones:
            return MainWindow.__offset_coords(current_lat, current_lon, heading, KAMIKAZE_RECOVER_LEAD)
        for fraction in KAMIKAZE_RECOVER_LEAD_FRACTIONS:
            distance = KAMIKAZE_RECOVER_LEAD * fraction
            for offset in KAMIKAZE_RECOVER_HEADING_OFFSETS:
                lead_lat, lead_lon = MainWindow.__offset_coords(
                    current_lat, current_lon, heading + offset, distance)
                if segment_hits_zone(current_lat, current_lon, lead_lat, lead_lon, zones) is None:
                    if offset != 0.0 or fraction != 1.0:
                        qDebug("Kurtarma hedefi HSS için kaydırıldı: %+.0f derece, %.0f m"
                               % (offset, distance))
                    return lead_lat, lead_lon
        self._create_warning("Kurtarma hattı HSS bölgesinden temizlenemedi, düz tırmanılıyor")
        return MainWindow.__offset_coords(current_lat, current_lon, heading, KAMIKAZE_RECOVER_LEAD)

    @staticmethod
    def __calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def __bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return math.atan2(x, y)

    @staticmethod
    def __offset_coords(latitude: float, longitude: float, heading: float, distance: float) -> tuple[float, float]:
        R = 6371000
        angular: float = distance / R
        bearing: float = math.radians(heading)
        phi1: float = math.radians(latitude)
        lambda1: float = math.radians(longitude)

        phi2 = math.asin(math.sin(phi1) * math.cos(angular) + math.cos(phi1) * math.sin(angular) * math.cos(bearing))
        lambda2 = lambda1 + math.atan2(
            math.sin(bearing) * math.sin(angular) * math.cos(phi1),
            math.cos(angular) - math.sin(phi1) * math.sin(phi2)
        )
        return math.degrees(phi2), math.degrees(lambda2)

    def __finish_kamikaze(self):
        """
        Koşuyu bitirir ve aracın sahipliğini baseline'a geri verir.

        Kamikaze'nin TEK çıkışı burasıdır: normal tamamlama, iptal, elle mod
        değişimi, dışarıdan mod değişimi, force-end, watchdog ve bağlantı
        kopması hepsi buraya gelir. Bağlantı yokken de çağrılabilir olması
        şart -- o durumda parametre yazılamaz, ama kilit yine de bırakılır ve
        yeniden bağlanınca apply_baseline_params() araca doğru durumu yazar.
        """
        self.kamikaze_timer.stop()
        # Önce sahiplik bırakılır, sonra yazılır: __set_param_group sahibe
        # bakıyor, sıra ters olsa baseline yazıları kendi kilidimize takılırdı.
        self.kamikaze_state = KamikazeState.IDLE
        self._kamikaze_target_cooldown.stop()
        # The run opened up the whole TECS envelope, the pitch floor, the roll
        # limit and the throttle ceiling so the dive could be flown in GUIDED.
        # One call puts all of it back -- see BASELINE_PARAMS.
        self.apply_baseline_params()
        self.ui.map_view.vehicle_locked = False
        if self.kamikaze_previous_mode is not None and self.kamikaze_previous_mode >= 0 and self.mavlink_connection is not None:
            self.mavlink_connection.set_mode_apm(self.kamikaze_previous_mode)
            self._mode_change_cooldown.start()
        if self.server_connection.ip is not None:
            self.on_kamikaze_end(self.kamikaze_qr_text)
        self.waits_for_qr = False
        # The numbers the dive is tuned against, on the status bar rather than
        # only in the log: the angle it actually reached, how long it had to
        # reach it, and where it bottomed out.
        if self.kamikaze_steepest_angle > 0.0:
            # Süre dalışın kendisi -- kurtarma tırmanışı dahil DEĞİL.
            qInfo("[Dive] ozet: %.0fm'den basladi, %.1f sn, en dik yol acisi %.1f, en dik pitch %.1f, hucum %.1f, dipte %.0fm"
                  % (self.kamikaze_dive_start_alt, self.kamikaze_dive_duration,
                     self.kamikaze_steepest_angle, self.kamikaze_steepest_pitch,
                     self.kamikaze_steepest_angle + self.kamikaze_steepest_pitch,
                     self.kamikaze_lowest_alt))
            self._create_warning("Dalış: en dik %.0f derece, %.1f sn, dip %.0fm (hedef %.0f derece / %.0fm)"
                                 % (self.kamikaze_steepest_angle, self.kamikaze_dive_duration,
                                    self.kamikaze_lowest_alt, KAMIKAZE_DIVE_ANGLE, KAMIKAZE_MIN_ALT))
        elif self.kamikaze_lowest_alt < math.inf:
            self._create_warning("Kamikaze bottomed out at %.0fm, %.0fm was asked for"
                                 % (self.kamikaze_lowest_alt, KAMIKAZE_MIN_ALT))
        self.kamikaze_lowest_alt = math.inf
        self.kamikaze_steepest_angle = 0.0
        self.kamikaze_steepest_pitch = 0.0
        self.kamikaze_sink_peaks = []
        self._kamikaze_climb_warned = False
        qDebug("Kamikaze Completed")
        # Koşu boyunca beklemeye alınan HSS işleri şimdi yapılabilir.
        self._flush_deferred_hss()

    def on_qr_found(self, qr_text: str):
        if not self.waits_for_qr:
            return
        self.waits_for_qr = False
        self.kamikaze_qr_text = qr_text
        if self.kamikaze_state == KamikazeState.DIVING and self.mavlink_connection is not None:
            qDebug("QR found during dive, switching to recovery")
            self.next_telemetry.lock.lockForRead()
            current_lat: float = self.next_telemetry.iha_enlem
            current_lon: float = self.next_telemetry.iha_boylam
            self.next_telemetry.lock.unlock()
            self.kamikaze_dive_duration = time.monotonic() - self.kamikaze_dive_started_at
            self.__enter_recovery(current_lat, current_lon)

    def on_kamikaze_end(self, qr_text: str) -> None:
        self.next_telemetry.lock.lockForRead()
        kamikaze_end = self.next_telemetry.gps_saati
        self.next_telemetry.lock.unlock()
        try:
            send_kamikaze(self.server_connection.get_address(), self.kamikaze_start, kamikaze_end, qr_text)
            qInfo("Kamikaze information sent with start: %s, end: %s, text: %s" % (self.kamikaze_start, kamikaze_end, qr_text))
        except Exception as e:
            self._create_warning("Could not send kamikaze info to server: %s" % e)

    def __force_end_task(self):
        if self.mavlink_connection is None:
            return
        qInfo("Force End Task requested by user")
        if self.ui.map_view.target_coord.is_set:
            self.ui.map_view.target_coord.remove_position()
        if self.kamikaze_state != KamikazeState.IDLE:
            self.kamikaze_previous_mode = 10  # resume the AUTO mission route
            # Baseline'ı __finish_kamikaze uyguluyor; burada ayrıca yazmak
            # kilide takılırdı ve zaten tekrar olurdu.
            self.__finish_kamikaze()
        else:
            self.apply_baseline_params()
            self.mavlink_connection.set_mode_apm(10)
            self._mode_change_cooldown.start()
            self.next_telemetry.lock.lockForWrite()
            self.next_telemetry.iha_otonom = 1
            self.next_telemetry.lock.unlock()
        self._create_warning("Task force-ended, returning to AUTO mission")

    def __get_kamikaze_coords(self):
        if self.server_connection.ip is None:
            self.ui.kamikaze_latitude.setText("39.90448632092518")
            self.ui.kamikaze_longitude.setText("41.23701348598452")
            return
        try:
            qr_coords: QrCoords = get_kamikaze_coords(self.server_connection.get_address())
        except Exception as e:
            self._create_warning("Could not get kamikaze coords from server: %s" % e)
            return
        if qr_coords is None:
            self._create_warning("Could not get kamikaze coords from server")
            return
        self.ui.kamikaze_latitude.setText(str(qr_coords.qrEnlem))
        self.ui.kamikaze_longitude.setText(str(qr_coords.qrBoylam))

    def _change_index(self, index: int):
        self._mode_change_cooldown.start()
        if self.kamikaze_state != KamikazeState.IDLE:
            # Only reached when the operator picks a mode by hand. The run has
            # to be called off here, otherwise it would drag the vehicle back
            # into GUIDED with its next destination update. The mode itself is
            # sent below, so the run must not restore the old one on its way
            # out.
            qDebug("Flight mode changed by hand, cancelling kamikaze")
            self.kamikaze_previous_mode = None
            self.__finish_kamikaze()
        if self.current_pilot == MAV_AUTOPILOT_PX4:
            base_mode = index_to_px4_uav_mode[index].value[2]
            sub_mode = index_to_px4_uav_mode[index].value[3]
            qDebug("Sending standard mode with mode %s, sub mode %s" % (base_mode, sub_mode))
            self.mavlink_connection.set_mode_px4(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, base_mode, sub_mode)
        else:
            qDebug("Sending mode with index: %s" % index)

            self.mavlink_connection.set_mode_apm(index)
            self.next_telemetry.lock.lockForWrite()
            self.next_telemetry.iha_otonom = 1 if index == 10 else 0
            self.next_telemetry.lock.unlock()

    def add_to_watch_list(self, e: TrackableDataEnum):
        if not TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].isEnabled():
            return
        rowCount: int = self.ui.watch_list.rowCount()
        self.ui.watch_list.setRowCount(rowCount + 1)

        self.ui.watch_list.setItem(rowCount, 0, QTableWidgetItem(str(e.value[0])))
        self.ui.watch_list.setItem(rowCount, 1, QTableWidgetItem(e.value[1]()))
        self.ui.watch_list.setItem(rowCount, 2, QTableWidgetItem(""))
        self.ui.watch_list.setItem(rowCount, 3, QTableWidgetItem(QCoreApplication.translate("TrackableDataEnum", "Unknown", None)))

        TRACKABLE_DATA_ENUM_ACTIONS[e.value[0]].setDisabled(True)

    def __remove_from_watch_signal(self):
        indexes: list[QModelIndex] = self.ui.watch_list.selectedIndexes()
        if indexes is not None:
            indexes.sort()

            rows: list[int] = list()
            i: int = 0
            for index in indexes:
                row: int = index.row()
                if row in rows:
                    continue
                rows.append(row)
                target_row: int = row - i

                if target_row > self.ui.watch_list.rowCount() or target_row < 0:
                    qWarning("You broke something, wrong target row found when trying to remove. Skipping row number %s" % target_row)
                    continue
                data_id = int(self.ui.watch_list.item(target_row, 0).text())

                TRACKABLE_DATA_ENUM_ACTIONS[data_id].setDisabled(False)
                self.ui.watch_list.removeRow(target_row)
                i = i + 1

    def _actionConfigurateServer(self):
        if self.server_connection_dialog is not None:
            return
        self.server_connection_dialog = ServerConnectionInterface(self)
        self.server_connection_dialog.show()
        if self.server_connection.ip is not None:
            self.server_connection_dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Server Connected :)", None))
            self.server_connection_dialog.ui.server_ip_input.setText(self.server_connection.ip)
            if self.server_connection.port:
                self.server_connection_dialog.ui.server_port_input.setText(str(self.server_connection.port))
            self.server_connection_dialog.ui.server_login_username_input.setText(str(self.server_connection.username))
            self.server_connection_dialog.ui.server_login_password_input.setText(str(self.server_connection.password))
        self.server_connection_dialog.ui.connect.clicked.connect(lambda: self._server_connect(self.server_connection_dialog))
        self.server_connection_dialog.ui.disconnect.clicked.connect(lambda button: self._server_disconnect())
        self.server_connection_dialog.finished.connect(lambda e: self._reset_dialog(False))

    def _actionConfigurateSetColors(self):
        if self.color_selector_dialog is not None:
            return
        self.color_selector_dialog = ColorSelectorInterface(self, self.color_options)
        self.color_selector_dialog.finished.connect(self._reset_color_configurate_screen)
        self.color_selector_dialog.show()

    def _reset_color_configurate_screen(self):
        self.color_options = self.color_selector_dialog.savedOptions
        self.color_selector_dialog = None

    def _actionConfigurate_UAV(self):
        if self.uav_connection_dialog is not None:
            return
        self.uav_connection_dialog = FightingUAVConnectionInterface(self)
        self.uav_connection_dialog.show()
        availablePorts = list(QSerialPortInfo.availablePorts())
        availablePorts.sort(key=lambda a: int(re.sub("\\D", "", a.portName())))
        for availablePort in availablePorts:
            self.uav_connection_dialog.ui.connection_type.addItem(availablePort.portName())
        if self.uav_connection.connection_type is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
            if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
                self.uav_connection_dialog.ui.serial_baud.setText(str(self.uav_connection.serial_baud_rate))
                idx = self.uav_connection_dialog.ui.connection_type.findText(self.uav_connection.serial_port)
                if idx >= 0:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(idx)
                else:
                    qWarning("Can not find index for serial port: %s" % self.uav_connection.serial_port)
            else:
                isTCP: bool = self.uav_connection_dialog.connection_type == ConnectionType.TCP
                self.uav_connection_dialog.ui.ip_address.setText(self.uav_connection.ip)
                if isTCP:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(0)
                else:
                    self.uav_connection_dialog.ui.connection_type.setCurrentIndex(1)
        self.uav_connection_dialog.ui.connect.clicked.connect(self._uav_connect)
        self.uav_connection_dialog.ui.disconnect.clicked.connect(self._uav_disconnect)
        self.uav_connection_dialog.finished.connect(lambda e: self._reset_dialog(True))

    def _reset_dialog(self, is_uav: bool):
        if is_uav:
            self.uav_connection_dialog = None
        else:
            self.server_connection_dialog = None

    connection_wait_wrapper: ConnectionWaitWrapper | None = None

    def _uav_connect(self):
        if self.uav_connection_dialog.connection_type != ConnectionType.SERIAL:
            if not self.uav_connection_dialog.ui.ip_address.text():
                self.uav_connection_dialog.ui.ip_address.setText(self.uav_connection_dialog.ui.ip_address.placeholderText())
            if self.is_ip_address_valid(self.uav_connection_dialog.ui.ip_address.text(), True):
                self.uav_connection_dialog.ui.invalid_input_error_label.hide()
            else:
                self.uav_connection_dialog.ui.invalid_input_error_label.show()
                return
        else:
            if not self.uav_connection_dialog.ui.serial_baud.text():
                self.uav_connection_dialog.ui.serial_baud.setText(self.uav_connection_dialog.ui.serial_baud.placeholderText())
            # TODO: Serial connection validation

        if self.uav_connection_dialog.connection_type == ConnectionType.SERIAL:
            self.uav_connection.serial_baud_rate = int(self.uav_connection_dialog.ui.serial_baud.text())
            self.uav_connection.serial_port = self.uav_connection_dialog.ui.connection_type.currentText()
        else:
            self.uav_connection.ip = self.uav_connection_dialog.ui.ip_address.text()

        if self.connection_wait_wrapper is not None:
            qWarning("Tried to press connect when trying to connect, ignoring")
            return
        self.uav_connection.connection_type = self.uav_connection_dialog.connection_type
        self.connection_wait_wrapper = ConnectionWaitWrapper(self, self.uav_connection)
        self.connection_wait_wrapper.setup_for_autopilot.connect(self.change_autopilot)
        self.connection_wait_wrapper.after_heartbeat_successfully_received.connect(self.__successful_uav_connection)
        self.connection_wait_wrapper.after_heartbeat_not_received.connect(self.__error_when_receiving_heartbeat)
        self.connection_wait_wrapper.set_device_connection_text.connect(self.uav_connection_dialog.ui.device_connection_text.setText)
        self.connection_wait_wrapper.mavlink_connection_error.connect(self.__uav_mav_connection_error)
        connection_thread: QThread = QThread(self)
        connection_thread.setObjectName("Connection Thread")
        connection_thread.started.connect(self.connection_wait_wrapper.run)
        self.connection_wait_wrapper.con_thread = connection_thread
        self.connection_wait_wrapper.moveToThread(connection_thread)
        connection_thread.start()

    def __uav_mav_connection_error(self):
        self.uav_connection.reset_connection_properties()

    def __error_when_receiving_heartbeat(self, mav_connection: mavfile):
        if self.uav_connection_dialog is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connection Failed :(", None))
            self.uav_connection_dialog.ui.invalid_input_error_label.show()

        if self.uav_connection.connection_type == ConnectionType.SERIAL:
            qInfo("Can not connect to UAV from %s" % (self.uav_connection.serial_port + "," + str(self.uav_connection.serial_baud_rate)))
        else:
            qInfo("Can not connect to UAV from %s" % self.uav_connection.ip)
        self.uav_connection.reset_connection_properties()
        self.ui.device_connection_warning.show()
        try:
            mav_connection.close()
        except:
            pass
        self.ui.map_view.mavlink_connection = None

    def __successful_uav_connection(self, mav_connection: mavfile):
        if self.uav_connection_dialog is not None:
            self.uav_connection_dialog.ui.device_connection_text.setText(QCoreApplication.translate("UAVConnection", "Device Connected :)", None))
        self.mavlink_connection = mav_connection
        self.mavlink_connection.mav.request_data_stream_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            MAV_DATA_STREAM_ALL,
            10,
            1
        )
        for e in TrackableDataPacketTimer:
            self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
                                                            self.mavlink_connection.target_component,
                                                            MAV_CMD_SET_MESSAGE_INTERVAL,
                                                            0,
                                                            e.value[0],
                                                            e.value[3],
                                                            0, 0, 0, 0, 0)
        self._enable_fence()
        self._update_time_with_mavlink()
        # Bağlantı kurulduğunda araç HER ZAMAN bilinen bir duruma çekilir. Bu,
        # uçuş ortasında telsiz kopup geri geldiğinde de çalışan tek kurtarma
        # yolu: koşu ortasında kopan bir kamikaze, dalış zarfını araçta bırakmış
        # olabilir ve burası onu kapatır. Değerlerin tanımı BASELINE_PARAMS ve
        # HSS_SAFETY_PARAMS tablolarında, burada literal yok.
        # Yazma işi worker thread'i başladıktan SONRA yapılıyor: teyit
        # (PARAM_VALUE) o thread üzerinden geliyor, önce yazarsak hepsi
        # cevapsız kalmış gibi görünüp boşuna tekrarlanırdı.

        self.ui.map_view.mavlink_connection = self.mavlink_connection
        self.mavlink_thread = QThread(self)
        self.mavlink_thread.setObjectName("Mavlink Connection Thread")
        self.mavlink_worker = MavlinkWorker(self.mavlink_connection, self)
        self.mavlink_worker.running = True
        self.mavlink_worker.update_watch_list.connect(self._apply_watch_update)
        self.mavlink_worker.create_warning.connect(self._create_warning)
        self.mavlink_worker.connection_lost.connect(self.__on_connection_lost)
        self.mavlink_worker.mission_fence_item_received.connect(self.mission_fence_item_received)
        self.mavlink_worker.mission_fence_item_int_received.connect(self.mission_fence_item_int_received)
        self.mavlink_worker.mission_waypoint_item_received.connect(self.mission_waypoint_item_received)
        self.mavlink_worker.mission_waypoint_item_int_received.connect(self.mission_waypoint_item_int_received)
        self.mavlink_worker.send_fence_mission_data.connect(self.send_fence_mission_data)
        self.mavlink_worker.remove_reposition_location.connect(self.remove_reposition_location)
        self.mavlink_worker.worker_signals.set_arm_mode.connect(self.set_arm_mode)
        self.mavlink_worker.worker_signals.set_fly_mode.connect(self.set_fly_mode)
        self.mavlink_worker.worker_signals.change_autopilot.connect(self.change_autopilot)
        self.mavlink_worker.worker_signals.should_reposition_removed.connect(self.should_reposition_removed)
        self.mavlink_thread.started.connect(self.mavlink_worker.run, Qt.ConnectionType.DirectConnection)
        self.mavlink_worker.mission_upload_success.connect(self._on_mission_upload_success)
        self.mavlink_worker.mission_upload_failed.connect(self._on_mission_upload_failed)
        self.mavlink_worker.mission_current_changed.connect(self._on_mission_current_changed)
        self.mavlink_worker.param_value_received.connect(self._on_param_value)
        self.mavlink_worker.moveToThread(self.mavlink_thread)
        self.mavlink_thread.start()
        self.apply_baseline_params()
        self.apply_hss_safety_params()
        self.enableFeaturesAfterUAVConnected()

    def remove_reposition_location(self):
        if self.ui.map_view.target_coord.is_set:
            self.ui.map_view.target_coord.remove_position()

    def should_reposition_removed(self):
        if self.ui.map_view.target_coord.is_set and not self.ui.map_view.reposition_timer.isActive():
            self.ui.map_view.target_coord.remove_position()
            if self.mavlink_connection is not None and self.kamikaze_state == KamikazeState.IDLE:
                # Reposition sadece THR_MAX ve roll limitini açıyor, ama
                # baseline'ın tamamını yazmak hem ucuz hem de "geri koymayı
                # unutulmuş bir parametre" ihtimalini tamamen kaldırıyor.
                self.apply_baseline_params()

    def set_arm_mode(self, index: int):
        self.uav_is_armed = bool(index)
        if not self._arm_change_cooldown.isActive() and self.ui.arm_mode.currentIndex() != index:
            self.ui.arm_mode.blockSignals(True)
            self.ui.arm_mode.setCurrentIndex(index)
            self.ui.arm_mode.blockSignals(False)

    def set_fly_mode(self, index: int):
        # Otopilotun bildirdiği gerçek mod. Kamikaze koşusu sürerken buraya
        # GUIDED dışında bir mod gelmesi, aracın kontrolünün dışarıdan alındığı
        # anlamına gelir: çit ihlali, failsafe veya RC pilotu. Koşu bunu
        # görmezden gelirse 2 saniyede bir DO_REPOSITION gönderip aracı GUIDED'a
        # geri çeker ve iki taraf mod salınımına girer -- eskiden aynen böyle
        # oluyordu, çünkü bu sinyal yolu combobox'ı bloke ederek güncelliyor ve
        # _change_index hiç çalışmıyor.
        if (self._kamikaze_owns_vehicle() and not self._mode_change_cooldown.isActive()
                and index != Ardupilot_UAV_Modes.GUIDED.value[0]):
            self._create_warning("Uçuş modu dışarıdan değişti (%s), kamikaze bırakılıyor" % index)
            qWarning("External mode change to %s during kamikaze, releasing the vehicle" % index)
            self.kamikaze_previous_mode = None  # dışarısı sürüyor, moda karışma
            self.__finish_kamikaze()
        if not self._mode_change_cooldown.isActive() and self.ui.fly_mode_combobox.currentIndex() != index:
            self.ui.fly_mode_combobox.blockSignals(True)
            self.ui.fly_mode_combobox.setCurrentIndex(index)
            self.ui.fly_mode_combobox.blockSignals(False)

    def _update_time_with_mavlink(self):
        time_ns = QDateTime.currentDateTimeUtc().toMSecsSinceEpoch() * 1000000
        time_ns += 1234 # Copied from mavproxy
        self.mavlink_connection.mav.timesync_send(0, time_ns)

    def __send_gcs_heartbeat(self) -> None:
        if self.mavlink_connection is None:
            return
        self.mavlink_connection.mav.heartbeat_send(
            MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

    def _apply_watch_update(self, row: int, value: str):
        self.ui.watch_list.setItem(row, 3, QTableWidgetItem(value))

    def _create_warning(self, text: str) -> None:
        qWarning(text)
        self.ui.statusbar.showMessage(text, 2000)

    def enableFeaturesAfterUAVConnected(self):
        self.ui.arm_mode.setEnabled(True)
        self.ui.fly_mode_combobox.setEnabled(True)
        self.ui.device_connection_warning.hide()
        self._gcs_heartbeat_timer.start()
        if self.server_connection.ip is None:
            self.plane_on_map_update_timer.start()

    def disableFeaturesAfterUAVDisconnected(self):
        self.ui.arm_mode.setEnabled(False)
        self.ui.fly_mode_combobox.setEnabled(False)
        self.ui.device_connection_warning.show()
        self._gcs_heartbeat_timer.stop()
        if self.plane_on_map_update_timer.isActive():
            self.plane_on_map_update_timer.stop()


    def __on_connection_lost(self, reason: str):
        self._create_warning("UAV connection lost: %s" % reason)
        if self.kamikaze_state != KamikazeState.IDLE:
            # Tam bir bırakma yapılıyor. Bağlantı gittiği için parametreler
            # araca yazılamaz -- araç dalış zarfında kalır. Yeniden bağlanınca
            # __successful_uav_connection baseline'ın TAMAMINI yazdığı için o
            # zarf orada kapanır. Eskiden burada sadece state sıfırlanıyordu ve
            # -60 derece pitch tabanı, kapalı glide slope, SPDWEIGHT=0 uçuşun
            # geri kalanı boyunca araçta kalıyordu.
            #
            # Ertelenmiş HSS işleri önce temizleniyor: __finish_kamikaze sonunda
            # onları uygulamaya çalışır ve ölü bir bağlantıya fence yüklemeye
            # kalkardı. Yeniden bağlanınca polling zaten yeni bir anlık görüntü
            # getiriyor.
            self._deferred_snapshot = None
            self._deferred_manual_ads = False
            self.__finish_kamikaze()
        self._param_pending.clear()
        self._param_retry_timer.stop()
        self._uav_disconnect()

    def _uav_disconnect(self):
        if self.mavlink_connection is None:
            return
        self.mavlink_worker.running = False
        self.mavlink_thread.quit()
        self.mavlink_thread.wait()
        try:
            self.mavlink_connection.close()
        except Exception as e:
            qWarning("Error while closing MAVLink connection: %s" % e)
        self.mavlink_connection = None
        self.ui.map_view.mavlink_connection = None
        self.uav_connection.reset_connection_properties()
        self.disableFeaturesAfterUAVDisconnected()
        self.uav_is_armed = False
        self.next_telemetry = TelemetryData()
        self.resetWatcherWidgetValues()
        self.ui.fly_mode_combobox.setCurrentIndex(-1)
        self.ui.arm_mode.setCurrentIndex(-1)

    def _server_connect(self, dialog: ServerConnectionInterface):
        if self.server_connection.ip:
            qDebug("Server already connected, disconnecting")
            self._server_disconnect()
        # TODO: Test connection
        if not dialog.ui.server_ip_input.text():
            dialog.ui.server_ip_input.setText(dialog.ui.server_ip_input.placeholderText())
        is_it_direct_address: bool = QRegularExpression("[\\w]+").match(dialog.ui.server_ip_input.text()).hasMatch()
        if not is_it_direct_address and not dialog.ui.server_port_input.text():
            dialog.ui.server_port_input.setText(dialog.ui.server_port_input.placeholderText())
        if not dialog.ui.server_login_username_input.text():
            dialog.ui.server_login_username_input.setText(dialog.ui.server_login_username_input.placeholderText())
        if not dialog.ui.server_login_password_input.text():
            dialog.ui.invalid_input_error_label.show()
            return
        dialog.ui.invalid_input_error_label.hide()
        self.server_connection.ip = dialog.ui.server_ip_input.text()
        if not ("://" in self.server_connection.ip):
            self.server_connection.ip = "http://"+self.server_connection.ip
        self.server_connection.username = dialog.ui.server_login_username_input.text()
        self.server_connection.password = dialog.ui.server_login_password_input.text()
        if len(dialog.ui.server_port_input.text()) != 0:
            self.server_connection.port = int(dialog.ui.server_port_input.text())
        else:
            self.server_connection.port = None

        try:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Trying to connect to server :O", None))
            self.server_connection.team_no = login_to_server(self.server_connection.get_address(), self.server_connection.username, self.server_connection.password)
            self.next_telemetry.lock.lockForWrite()
            self.next_telemetry.takim_numarasi = self.server_connection.team_no
            self.next_telemetry.lock.unlock()
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Server Connected :)", None))
            self.ui.server_connection_warning.hide()
            qInfo("Connected to server with ip: %s, username: %s" % (self.server_connection.get_address(), self.server_connection.username))
        except Exception as e:
            dialog.ui.server_connection_text.setText(QCoreApplication.translate("ServerConfig", "Can't Connect To Server :(", None))
            qWarning("Can not connect to server: %s" % e)
            dialog.ui.invalid_input_error_label.show()
            self.ui.server_connection_warning.show()
            self.server_connection.ip = None
            self.server_connection.port = None
            return
        if self.plane_on_map_update_timer.isActive():
            self.plane_on_map_update_timer.stop()
        self.server_connection.telemetry_timer = QTimer()
        self.server_connection.telemetry_timer.setInterval(700)
        self.server_connection.telemetry_timer.timeout.connect(self.__send_telemetry, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread = QThread(self)
        self.server_connection.telemetry_thread.setObjectName("Telemetry Thread")
        self.server_connection.telemetry_timer.moveToThread(self.server_connection.telemetry_thread)
        self.server_connection.telemetry_thread.started.connect(self.server_connection.telemetry_timer.start, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread.finished.connect(self.server_connection.telemetry_timer.stop, type=Qt.ConnectionType.DirectConnection)
        self.server_connection.telemetry_thread.start()

        # HSS otomatik polling başlat
        self._start_hss_polling()

    def _start_hss_polling(self) -> None:
        """Sunucu bağlantısı kurulduğunda HSS polling worker'ı başlat."""
        if self._hss_polling_worker is not None:
            return  # Zaten çalışıyor
        worker = HSSPollingWorker(self.server_connection.get_address())
        thread = QThread(self)
        thread.setObjectName("HSS Polling Thread")
        worker.moveToThread(thread)
        worker.hss_updated.connect(self._on_hss_updated)
        worker.hss_error.connect(lambda msg: qDebug("[HSS] %s" % msg))
        thread.started.connect(worker.run)
        thread.finished.connect(worker.stop)
        self._hss_polling_worker = worker
        self._hss_polling_thread = thread
        thread.start()
        qDebug("[HSS] Polling started")

    def _stop_hss_polling(self) -> None:
        if self._hss_polling_thread is not None:
            self._hss_polling_thread.quit()
            self._hss_polling_thread.wait()
        self._hss_polling_worker = None
        self._hss_polling_thread = None
        qDebug("[HSS] Polling stopped")

    def _hss_zones(self) -> list[ServerAdsData]:
        """
        Sunucu + manuel HSS bölgelerinin tek listesi (ham yarıçaplarla).

        Hem rota planlayıcı, hem çit yüklemesi, hem de kamikaze giriş kapısı
        buradan besleniyor -- "hangi bölgeler var" sorusunun tek cevabı olsun
        diye. Tampon ekleme işi tüketiciye ait (fence_radius_for_hss).
        """
        zones: list[ServerAdsData] = list(self._current_snapshot.zones) if self._current_snapshot else []
        for ads in self.ui.map_view.user_ads_data_model.m_datas:
            zones.append(ServerAdsData(
                id=-1,
                lat=ads.position.latitude(),
                lon=ads.position.longitude(),
                radius_m=ads.size
            ))
        return zones

    def _build_fence_ads_list(self) -> list[AdsData]:
        """
        Araca yüklenecek exclusion çemberleri — HER ZAMAN tamponlu.

        Eskiden iki ayrı yol vardı: biri server_ads_data_model'deki HAM
        yarıçapları yüklüyordu (harita için doğru, çit için 20 m dar), diğeri
        tamponluyu. İkisi arka arkaya tetiklendiği için her HSS güncellemesinde
        telsizden iki fence yüklemesi geçiyor ve arada araçta dar çit kalıyordu.
        Artık tek tanım burası.
        """
        ads_list: list[AdsData] = []
        for hss in self._hss_zones():
            ads = AdsData()
            ads.position = QGeoCoordinate(hss.lat, hss.lon)
            ads.size = fence_radius_for_hss(hss.radius_m)
            ads.is_selected = False
            ads_list.append(ads)
        return ads_list

    def _on_hss_updated(self, snapshot: HssSnapshot) -> None:
        """Yeni HSS listesi geldiğinde tetiklenir (HSS polling thread'den sinyal)."""
        # Out-of-order koruması
        if self._current_snapshot is not None and snapshot.seq <= self._current_snapshot.seq:
            qDebug("[HSS] Stale snapshot (seq=%d <= %d), skipping" % (snapshot.seq, self._current_snapshot.seq))
            return

        self._current_snapshot = snapshot
        zones = list(snapshot.zones)

        # 1. Haritayı güncelle (kırmızı HSS çemberleri). notify=False: çiti
        #    aşağıda kendimiz, tamponlu yarıçaplarla yüklüyoruz.
        self.ui.map_view.update_server_ads_data(zones, notify=False)
        # 2. Tampon bölge çemberlerini güncelle (turuncu)
        self._update_buffer_zones(zones)

        # 3-4. Çit yüklemesi ve rota planlaması araca komut vermek demek.
        #      Kamikaze koşusu sürerken bu iş yapılmaz: fence yüklemesi telsizi
        #      doldurup kritik parametre yazılarının düşmesine yol açar, rota
        #      yüklemesi de koşu bitince dönülecek AUTO görevini altından
        #      değiştirir. Anlık görüntü saklanır, koşu biter bitmez uygulanır.
        if self._kamikaze_owns_vehicle():
            first_deferral = self._deferred_snapshot is None
            self._deferred_snapshot = snapshot
            qDebug("[HSS] Kamikaze koşusu sürüyor, %d bölgelik güncelleme ertelendi" % len(zones))
            if first_deferral:
                # Koşu boyunca 3 saniyede bir tekrarlamasın; durum çubuğu
                # kamikaze'nin kendi uyarıları için lazım.
                self._create_warning("Kamikaze sürüyor — HSS güncellemesi koşu sonuna ertelendi")
            return

        self._auto_upload_hss_fences()
        self._trigger_route_replanning()

    def _flush_deferred_hss(self) -> None:
        """Kamikaze koşusu boyunca beklemeye alınan HSS işlerini uygular."""
        if self._kamikaze_owns_vehicle():
            return
        if self._deferred_snapshot is None and not self._deferred_manual_ads:
            return
        qDebug("[HSS] Ertelenmiş güncelleme uygulanıyor")
        self._deferred_snapshot = None
        self._deferred_manual_ads = False
        self._auto_upload_hss_fences()
        # Koşu sırasında bölge listesi değişmemiş olsa bile yeniden planla:
        # araç koşu boyunca görev rotasının dışına çıktı.
        self._last_planned_hash = None
        self._trigger_route_replanning()

    def _update_buffer_zones(self, zones: list[ServerAdsData]) -> None:
        """Tampon bölge çemberlerini haritada güncelle (turuncu, R_hss + R_turn)."""
        model = self.ui.map_view.buffer_zone_data_model
        model.m_datas.clear()
        for hss in zones:
            ads = AdsData()
            ads.position = QGeoCoordinate(hss.lat, hss.lon)
            ads.size = fence_radius_for_hss(hss.radius_m)  # R_hss + R_turn
            ads.is_selected = False
            model.m_datas.append(ads)
        model.layoutChanged.emit()

    def _auto_upload_hss_fences(self) -> None:
        """HSS listesini exclusion circle olarak ArduPilot'a yükler (R_hss + FENCE_BUFFER_M)."""
        if self.mavlink_connection is None:
            return
        if self._kamikaze_owns_vehicle():
            self._deferred_manual_ads = True
            return
        self._fence_upload_retries = 0
        self.update_geofence_data(self._build_fence_ads_list())
        # _enable_fence artık yükleme BİTTİĞİNDE çağrılıyor (bkz.
        # send_fence_mission_data); burada çağrılması, henüz yüklenmemiş bir
        # çiti aktif etmek anlamına geliyordu.

    def _trigger_route_replanning(self) -> None:
        """Mission + HSS verisi mevcutsa rota yeniden hesapla ve haritada göster."""
        if not self._current_mission_waypoints:
            return
        if self._kamikaze_owns_vehicle():
            self._deferred_manual_ads = True
            return
        if self._plan_in_flight:
            # Havuzda zaten bir hesap var; hash sıfırlanarak bittiğinde
            # yenisinin tetiklenmesi sağlanır.
            self._last_planned_hash = None
            return

        combined_hss: list = self._hss_zones()
        if not combined_hss:
            return

        # Zone hash değişmediyse yeniden planlamaya gerek yok
        current_hash = hash(tuple(combined_hss))
        if self._last_planned_hash == current_hash:
            return
        self._last_planned_hash = current_hash

        # Hesap havuz thread'inde yapılıyor; sonuç _on_route_plan_ready'e
        # sinyalle geliyor. Bkz. RoutePlanTask.
        self._plan_in_flight = True
        QThreadPool.globalInstance().start(RoutePlanTask(
            list(self._current_mission_waypoints), combined_hss,
            current_hash, self._route_plan_signals))

    def _on_route_plan_ready(self, result: RoutePlanResult | None, plan_hash) -> None:
        self._plan_in_flight = False
        if result is None:
            self._last_planned_hash = None   # bir dahaki tetikte tekrar denensin
            self._create_warning("Rota hesabı başarısız oldu, düzeltilmiş rota güncellenmedi")
            return
        # Hesap sürerken kamikaze başlamış olabilir: sonuç bayatlamadı ama
        # şu an araca yüklenemez, koşu sonunda yeniden planlanır.
        if self._kamikaze_owns_vehicle():
            self._deferred_manual_ads = True
            self._last_planned_hash = None
            return
        if self._last_planned_hash is None:
            # Hesap sürerken bölgeler değişti — sonucu göster ama hemen
            # güncelini tetikle.
            QTimer.singleShot(0, self._trigger_route_replanning)
        # Yeşil alternatif rotayı haritada güncelle
        avoidance_geopath = self.ui.map_view.avoidance_route_geopath
        avoidance_geopath.clear()
        for rp in result.corrected_waypoints:
            coord = QGeoCoordinate(rp.lat, rp.lon)
            coord.setAltitude(rp.alt)
            avoidance_geopath.add_pos(coord)
        avoidance_geopath.mission_geopath_changed.emit()

        # Çakışma uyarıları
        if result.has_conflicts:
            self._create_warning(
                "HSS çakışması tespit edildi! Bölgeler: %s" % result.conflict_zone_ids
            )
        if result.waypoints_inside_zone:
            self._create_warning(
                "KRİTİK: WP #%s HSS bölgesi içinde — operatör müdahalesi gerekiyor!"
                % result.waypoints_inside_zone
            )
        qDebug("[RoutePreplanner] Corrected route: %d waypoints, conflicts: %s"
               % (len(result.corrected_waypoints), result.conflict_zone_ids))

        # Otomatik yükleme: çakışma varsa direkt ArduPilot'a gönder
        if result.has_conflicts and self.mavlink_connection is not None:
            qDebug("[RoutePreplanner] Çakışma var, düzeltilmiş rota otomatik yükleniyor!")
            self._upload_corrected_route()

    def __update_plane_on_map_without_server(self):
        if self.mavlink_connection is None or self.server_connection.ip is not None:
            self.plane_on_map_update_timer.stop()
            return
        self.next_telemetry.lock.lockForRead()
        enlem: float = self.next_telemetry.iha_enlem
        boylam: float = self.next_telemetry.iha_boylam
        yaw: float = self.next_telemetry.iha_yonelme
        self.next_telemetry.lock.unlock()
        self.ui.map_view.update_plane_data_without_server(QGeoCoordinate(enlem, boylam), yaw)

    def __send_telemetry(self):
        if self.mavlink_connection is None:
            qDebug("UAV not connected")
            return
        if server_api.SERVER_IS_UNREACHABLE_COUNTER > 100:
            server_api.SERVER_IS_UNREACHABLE_COUNTER = 0
            qWarning("Server connection is not possible for 100 time, disconnecting")
            self._server_disconnect()
            return
        self.next_telemetry.lock.lockForRead()
        try:
            telemetry_snapshot = copy.copy(self.next_telemetry)
        finally:
            self.next_telemetry.lock.unlock()
        qDebug("Sending telemetry at %s" % QDateTime.currentDateTime().toString())
        response = send_telemetry(self.server_connection.get_address(), telemetry_snapshot)
        if response:
            self.last_server_telemetry_response = response
            self.update_plane_data_signal.emit(telemetry_snapshot, response)
        else:
            qWarning("Could not process telemetry response info")
    update_plane_data_signal = Signal(TelemetryData, TelemetryResponseData)
    is_processing_plane_data = False
    def __update_plane_data(self, our_data: TelemetryData, telemetry_response: TelemetryResponseData):
        if self.is_processing_plane_data:
            return
        self.is_processing_plane_data = True
        try:
            self.ui.map_view.update_plane_data(our_data, telemetry_response)
        finally:
            self.is_processing_plane_data = False

    def _server_disconnect(self):
        if self.server_connection.telemetry_thread:
            self.server_connection.telemetry_thread.quit()
            self.server_connection.telemetry_thread.wait()
            self.server_connection.telemetry_thread.deleteLater()
            self.server_connection.telemetry_thread = None
        if self.server_connection.ip is None:
            return
        self.server_connection.ip = None
        self.server_connection.port = None
        qInfo("Disconnected from server")
        
        self._stop_hss_polling()
        self._reset_hss_state()

    def _reset_hss_state(self) -> None:
        """Tüm HSS durumunu sıfırla — tek yerde, ileride yeni katman eklenince buraya eklenir."""
        self._current_snapshot = None
        self._deferred_snapshot = None
        self._deferred_manual_ads = False
        self._last_planned_hash = None
        # Harita katmanlarını temizle
        self.ui.map_view.server_ads_data_model.m_datas.clear()
        self.ui.map_view.server_ads_data_model.layoutChanged.emit()
        self.ui.map_view.buffer_zone_data_model.m_datas.clear()
        self.ui.map_view.buffer_zone_data_model.layoutChanged.emit()
        self.ui.map_view.avoidance_route_geopath.clear()
        self.ui.map_view.avoidance_route_geopath.mission_geopath_changed.emit()

    @staticmethod
    def is_ip_address_valid(ip_address: str, must_have_port: bool) -> bool:
        if ip_address is not None:
            ip_address = ip_address.strip()
            ip_with_port: str = ip_address
            split: list[str] = ip_with_port.split(':')
            ip: str = split[0]
            if len(split) > 1:
                try:
                    port: int = int(split[1])
                    if port < 0 or port > 65535:
                        return False
                except:
                    return False
            elif must_have_port:
                return False # Must have port, yes that's the comment :3
            ip_array: list[str] = ip.split('.')
            if len(ip_array) != 4:
                return False
            for e in ip_array:
                try:
                    e: int = int(e)
                    if e < 0 or e > 255:
                        return False
                except:
                    return False
            return True
        return False

    def _upload_corrected_route(self) -> None:
        """Yeşil kaçınma rotasını MAVLink mission protokolü ile Pixhawk'a yükler."""
        if self.mavlink_connection is None:
            self._create_warning("UAV bağlı değil, rota yüklenemiyor")
            return
        if self._kamikaze_owns_vehicle():
            # Koşu sırasında görevi değiştirmek, koşu bitince dönülecek AUTO
            # rotasını ve current seq'i altından değiştirmek demek.
            self._deferred_manual_ads = True
            self._create_warning("Kamikaze sürüyor — rota yüklemesi koşu sonuna ertelendi")
            return

        # Seyir hızı artık BASELINE_PARAMS'ın parçası ve rota planlayıcının
        # varsaydığı hızla (RoutePreplanner.CRUISE_SPEED_MS) aynı. Buradaki eski
        # tek seferlik TRIM_ARSPD_CM=1500 yazısı hiçbir zaman geri alınmıyordu,
        # planlayıcının kendi tampon hesabıyla da çelişiyordu.

        if self.mavlink_worker._mission_upload_state > 0:
            self._create_warning("Rota yükleme devam ediyor, lütfen bekleyin")
            return

        # Yeşil rotanın waypoint'lerini al
        corrected_coords = []
        geopath = self.ui.map_view.avoidance_route_geopath.mission_geopath_v
        for i in range(geopath.size()):
            corrected_coords.append(geopath.coordinateAt(i))

        if not corrected_coords:
            self._create_warning("Düzeltilmiş rota bulunamadı, önce görev indirin ve HSS verisinin gelmesini bekleyin")
            return

        # İrtifa doğrulama — Home (0) ve son nokta (muhtemel iniş) haricindeki sıfır veya negatif irtifaları engelle
        for i, p in enumerate(corrected_coords):
            if p.altitude() <= 0:
                if i == 0 or i == len(corrected_coords) - 1:
                    continue
                self._create_warning("HATA: Rotada (WP %d) irtifası 0 olan nokta var, yükleme iptal!" % i)
                qWarning("[MissionUpload] Altitude validation failed at WP %d" % i)
                return

        self.mavlink_worker._mission_upload_items = corrected_coords
        self.mavlink_worker._start_mission_upload = True

        qDebug("[MissionUpload] Delegated corrected route upload to worker thread: %d waypoints" % len(corrected_coords))
        self._create_warning("Düzeltilmiş rota arka planda yükleniyor (%d waypoint)..." % len(corrected_coords))

    def _on_mission_upload_success(self, count: int) -> None:
        """MavlinkWorker arka planda rota yüklemeyi bitirdiğinde çağrılır."""
        self._create_warning(f"Düzeltilmiş rota başarıyla yüklendi! ({count} waypoint)")
        qDebug(f"[MissionUpload] SUCCESS for {count} waypoints")

        # Havadayken yeni rotanın sıradaki waypoint'inden devam et
        if self._is_airborne():
            resume_seq = self._find_resume_seq()
            if resume_seq > 0:
                self.mavlink_connection.mav.mission_set_current_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    resume_seq
                )
                qDebug("[MissionUpload] Airborne! SET_CURRENT to seq=%d" % resume_seq)
                self._create_warning("Havada rota güncellendi — WP %d'den devam ediliyor" % resume_seq)
        else:
            qDebug("[MissionUpload] On ground — mission starts from seq=0")

        self._verify_uploaded_mission()

    def _on_mission_upload_failed(self, reason: str) -> None:
        self._create_warning(f"Rota yükleme başarısız: {reason}")
        qWarning(f"[MissionUpload] FAILED: {reason}")

    def _on_mission_current_changed(self, seq: int) -> None:
        self._pixhawk_current_seq = seq

    def _is_airborne(self) -> bool:
        if self.mavlink_connection is None:
            return False
        armed = self.ui.arm_mode.currentIndex() == 1
        self.next_telemetry.lock.lockForRead()
        try:
            alt = self.next_telemetry.iha_irtifa
        finally:
            self.next_telemetry.lock.unlock()
        return armed and alt > 5.0

    def _find_resume_seq(self) -> int:
        """
        Yeni rotada uçağın devam etmesi gereken waypoint seq numarasını bul.
        Eski rotadaki current_seq'in karşılığını yeni rotada origin_idx üzerinden eşleştir.
        """
        old_seq = self._pixhawk_current_seq
        if old_seq <= 0:
            return 1  # Fallback: ilk waypoint'ten başla

        # Yeni rotadaki corrected_waypoints bilgisinden origin_idx eşleştirmesi yap
        geopath = self.ui.map_view.avoidance_route_geopath.mission_geopath_v
        new_count = geopath.size()

        # origin_idx bilgisi RoutePoint'te tutuluyor, ama geopath'te sadece koordinat var.
        # En güvenli yaklaşım: eski rotadaki WP koordinatını yeni rotada bul
        # ve o noktanın seq'ine veya ondan önceki bypass noktasının seq'ine SET_CURRENT yap.

        # Eski rotadaki current_seq'in koordinatını al
        if old_seq < len(self._current_mission_waypoints):
            target_coord = self._current_mission_waypoints[old_seq]
        else:
            return 1  # Fallback

        # Yeni rotada bu koordinatla eşleşen (veya ondan önceki bypass) noktayı bul
        best_seq = 1
        for i in range(new_count):
            coord = geopath.coordinateAt(i)
            dist = target_coord.distanceTo(coord)
            if dist < 50.0:  # 50m içinde eşleşme — bypass noktaları bu WP'den önce
                # Bu WP'den önceki ilk bypass noktasını bul
                best_seq = max(1, i)
                # Önceki noktalardan bypass olanları da dahil etmek için geriye git
                while best_seq > 1:
                    prev_coord = geopath.coordinateAt(best_seq - 1)
                    prev_dist = target_coord.distanceTo(prev_coord)
                    if prev_dist < target_coord.distanceTo(geopath.coordinateAt(0)):
                        best_seq -= 1
                    else:
                        break
                break

        qDebug("[MissionUpload] Resume: old_seq=%d -> new_seq=%d" % (old_seq, best_seq))
        return best_seq

    def _verify_uploaded_mission(self) -> None:
        """Upload sonrası görevi geri indirip waypoint sayısı + koordinat doğrulaması yapar."""
        if self.mavlink_connection is None:
            return
        # request_mission_data üzerinden gidiyor: eskiden mission_request_list
        # doğrudan gönderiliyordu ama requested_to_get_mission bayrağı set
        # edilmediği için worker gelen MISSION_COUNT'u atıyordu, yani doğrulama
        # hiçbir zaman çalışmıyordu.
        if self.requested_to_get_mission:
            qWarning("[MissionUpload] Görev indirmesi zaten sürüyor, doğrulama atlandı")
            return
        self._awaiting_mission_verification = True
        self.request_mission_data()
        qDebug("[MissionUpload] Verification download requested")

    def _on_manual_ads_changed(self) -> None:
        """Manuel ADS ekleme/silme — fence güncelle + rotayı yeniden hesapla."""
        if self._kamikaze_owns_vehicle():
            self._deferred_manual_ads = True
            self._create_warning("Kamikaze sürüyor — ADS değişikliği koşu sonuna ertelendi")
            return
        # Çit listesi tek yerden üretiliyor (her zaman tamponlu yarıçaplarla).
        self._auto_upload_hss_fences()
        self._last_planned_hash = None
        self._trigger_route_replanning()

