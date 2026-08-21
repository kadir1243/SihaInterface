from dataclasses import dataclass
from enum import Enum, IntEnum

from PySide6.QtCore import QCoreApplication, QLocale
from pymavlink.dialects.v20.all import MAVLink_battery_status_message, MAVLink_gps_raw_int_message, \
    MAVLink_attitude_message, MAVLink_vfr_hud_message, MAVLink_heartbeat_message, MAVLink_global_position_int_message, \
    MAVLink_system_time_message, MAVLink_fence_status_message

from src.ServerConnection import ServerAdsData


class KamikazeState(Enum):
    IDLE = 0
    APPROACHING = 1
    DIVING = 2
    RECOVERING = 3
    RESUMING = 4

class TrackableDataPacketTimer(Enum):
    # (msg id, msg name, type, update interval (microsecond), watch value ids that uses this packet)
    BATTERY_STATUS = (147, "BATTERY_STATUS", MAVLink_battery_status_message, 500000, [10])
    ATTITUDE = (30, "ATTITUDE", MAVLink_attitude_message, 250000, [3, 4, 5])
    GPS_RAW_INT = (24, "GPS_RAW_INT", MAVLink_gps_raw_int_message, 500000, [1, 8, 9])
    VFR_HUD = (74, "VFR_HUD", MAVLink_vfr_hud_message, 250000, [0, 6])
    HEARTBEAT = (0, "HEARTBEAT", MAVLink_heartbeat_message, 1000000, [11, 14])
    GLOBAL_POSITION_INT = (33, "GLOBAL_POSITION_INT", MAVLink_global_position_int_message, 250000, [2, 12])
    SYSTEM_TIME = (2, "SYSTEM_TIME", MAVLink_system_time_message, 500000, [7])
    FENCE_STATUS = (162, "FENCE_STATUS", MAVLink_fence_status_message, 500000, [13])

MSG_ID_2_TRACKABLE_DATA_TYPE: dict[int, TrackableDataPacketTimer] = {}

for ____trackable_data_packet_timer in TrackableDataPacketTimer:
    MSG_ID_2_TRACKABLE_DATA_TYPE[____trackable_data_packet_timer.value[0]] = ____trackable_data_packet_timer

class PX4_CUSTOM_MAIN_MODE(IntEnum):
    MANUAL = 1
    ALTCTL = 2
    POSCTL = 3
    AUTO = 4
    ACRO = 5
    OFFBOARD = 6
    STABILIZED = 7
    RATTITUDE_LEGACY = 8
    SIMPLE = 9 #/* unused, but reserved for future use */
    TERMINATION = 10
    ALTITUDE_CRUISE = 11

class PX4_CUSTOM_SUB_MODE(IntEnum):
    AUTO_READY = 1
    AUTO_TAKEOFF = 2
    AUTO_LOITER = 3
    AUTO_MISSION = 4
    AUTO_RTL = 5
    AUTO_LAND = 6
    AUTO_RESERVED_DO_NOT_USE = 7 #// was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
    AUTO_FOLLOW_TARGET = 8
    AUTO_PRECLAND = 9
    AUTO_VTOL_TAKEOFF = 10
    EXTERNAL1 = 11
    EXTERNAL2 = 12
    EXTERNAL3 = 13
    EXTERNAL4 = 14
    EXTERNAL5 = 15
    EXTERNAL6 = 16
    EXTERNAL7 = 17
    EXTERNAL8 = 18
    GUIDED_COURSE = 19

class PX4_CUSTOM_SUB_MODE_POSCTL(Enum):
    POSCTL = 0
    ORBIT = 1
    SLOW = 2

class PX4_UAV_Modes(Enum):
    MANUAL = (0, "MANUAL", PX4_CUSTOM_MAIN_MODE.MANUAL, 0)  # Manual mode
    ALTCTL = (1, "ALTCTL", PX4_CUSTOM_MAIN_MODE.ALTCTL, 0)  # Altitude control mode
    POSCTL = (2, "POSCTL", PX4_CUSTOM_MAIN_MODE.POSCTL, 0)  # Position control mode
    AUTO_MISSION = (3, "AUTO_MISSION", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_MISSION)  # Auto mission mode
    AUTO_LOITER = (4, "AUTO_LOITER", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_LOITER)  # Auto loiter mode
    AUTO_RTL = (5, "AUTO_RTL", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_RTL)  # Auto return to launch mode
    POSITION_SLOW = (6, "POSITION_SLOW", PX4_CUSTOM_MAIN_MODE.POSCTL, PX4_CUSTOM_SUB_MODE_POSCTL.SLOW)
    GUIDED_COURSE = (7, "GUIDED_COURSE", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.GUIDED_COURSE)  # Guided Course mode (FW: maintain course/alt/speed)
    ALTITUDE_CRUISE = (8, "ALTITUDE_CRUISE", PX4_CUSTOM_MAIN_MODE.ALTITUDE_CRUISE, 0)  # Altitude with Cruise mode
    FREE3 = (9, "FREE3", 0, 0)
    ACRO = (10, "ACRO", PX4_CUSTOM_MAIN_MODE.ACRO, 0)  # Acro mode
    FREE2 = (11, "FREE2", 0, 0)
    DESCEND = (12, "DESCEND", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_LAND)  # Descend mode (no position control)
    TERMINATION = (13, "TERMINATION", PX4_CUSTOM_MAIN_MODE.TERMINATION, 0)  # Termination mode
    OFFBOARD = (14, "OFFBOARD", PX4_CUSTOM_MAIN_MODE.OFFBOARD, 0)
    STAB = (15, "STAB", PX4_CUSTOM_MAIN_MODE.STABILIZED, 0)  # Stabilized mode
    FREE1 = (16, "FREE1", 0, 0)
    AUTO_TAKEOFF = (17, "AUTO_TAKEOFF", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_TAKEOFF)  # Takeoff
    AUTO_LAND = (18, "AUTO_LAND", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_LAND)  # Land
    AUTO_FOLLOW_TARGET = (19, "AUTO_FOLLOW_TARGET", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_FOLLOW_TARGET)  # Auto Follow
    AUTO_PRECLAND = (20, "AUTO_PRECLAND", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_PRECLAND)  # Precision land with landing target
    ORBIT = (21, "ORBIT", PX4_CUSTOM_MAIN_MODE.POSCTL, PX4_CUSTOM_SUB_MODE_POSCTL.ORBIT)  # Orbit in a circle
    AUTO_VTOL_TAKEOFF = (22, "AUTO_VTOL_TAKEOFF", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.AUTO_VTOL_TAKEOFF)  # Takeoff, transition, establish loiter
    EXTERNAL1 = (23, "EXTERNAL1", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL1)
    EXTERNAL2 = (24, "EXTERNAL2", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL2)
    EXTERNAL3 = (25, "EXTERNAL3", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL3)
    EXTERNAL4 = (26, "EXTERNAL4", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL4)
    EXTERNAL5 = (27, "EXTERNAL5", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL5)
    EXTERNAL6 = (28, "EXTERNAL6", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL6)
    EXTERNAL7 = (29, "EXTERNAL7", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL7)
    EXTERNAL8 = (30, "EXTERNAL8", PX4_CUSTOM_MAIN_MODE.AUTO, PX4_CUSTOM_SUB_MODE.EXTERNAL8)

main_and_sub_mode_to_px4_uav_mode: dict[int, dict[int, PX4_UAV_Modes]] = dict()

for __px4_uav_mode in PX4_UAV_Modes:
    m = main_and_sub_mode_to_px4_uav_mode.get(__px4_uav_mode.value[2])
    if not m:
        m = dict()
    m[__px4_uav_mode.value[3]] = __px4_uav_mode
    main_and_sub_mode_to_px4_uav_mode[__px4_uav_mode.value[2]] = m

index_to_px4_uav_mode: dict[int, PX4_UAV_Modes] = dict()
for ____px4_uav_mode in PX4_UAV_Modes:
    index_to_px4_uav_mode[____px4_uav_mode.value[0]] = ____px4_uav_mode

class Ardupilot_UAV_Modes(Enum):
    MANUAL = (0, 'MANUAL', 0)
    CIRCLE = (1, 'CIRCLE', 1)
    STABILIZE = (2, 'STABILIZE', 2)
    TRAINING = (3, 'TRAINING', 3)
    ACRO = (4, 'ACRO', 4)
    FLY_BY_WIRE_A = (5, 'FBWA', 5)
    FLY_BY_WIRE_B = (6, 'FBWB', 6)
    CRUISE = (7, 'CRUISE', 7)
    AUTOTUNE = (8, 'AUTOTUNE', 8)
    RESERVED_NON_SELECTABLE = (9, 'FREE_NON_SELECTABLE_INTERNAL', -1)
    AUTO = (10, 'AUTO', 10)
    ReturnToLaunch = (11, 'RTL', 11)
    LOITER = (12, 'LOITER', 12)
    TAKEOFF = (13, 'TAKEOFF', 13)
    AVOID_ADSB = (14, 'AVOID_ADSB', 14)
    GUIDED = (15, 'GUIDED', 15)
    INITIALISING = (16, 'INITIALISING', 16)
    QSTABILIZE = (17, 'QSTABILIZE', 17)
    QHOVER = (18, 'QHOVER', 18)
    QLOITER = (19, 'QLOITER', 19)
    QLAND = (20, 'QLAND', 20)
    QRTL = (21, 'QRTL', 21)
    QAUTOTUNE = (22, 'QAUTOTUNE', 22)
    QACRO = (23, 'QACRO', 23)
    THERMAL = (24, 'THERMAL', 24)
    LOITERALTQLAND = (25, 'LOITERALTQLAND', 25)
    AUTOLAND = (26, 'AUTOLAND', 26)

class SupportedLanguages(Enum):
    English = (0, lambda: QCoreApplication.translate("SupportedLanguages", "English", None), QLocale.Language.English, QLocale.Country.UnitedStates)
    Turkish = (1, lambda: QCoreApplication.translate("SupportedLanguages", "Turkish", None), QLocale.Language.Turkish, QLocale.Country.Turkey)
    @staticmethod
    def from_id(i: int) -> SupportedLanguages:
        e: SupportedLanguages
        for e in SupportedLanguages:
            if e.value[0] == i:
                return e
        return None

@dataclass(frozen=True, slots=True)
class HssSnapshot:
    seq: int
    zones: list[ServerAdsData]
