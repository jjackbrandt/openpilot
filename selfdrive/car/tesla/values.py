from collections import namedtuple

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarDocs

# Try to import fw_query_definitions, but make it optional for basic functionality
try:
  from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
  HAS_FW_QUERY = True
except ImportError:
  # Define dummy classes for when panda dependencies aren't available
  class FwQueryConfig: 
    def __init__(self, *args, **kwargs): pass
  class Request: 
    def __init__(self, *args, **kwargs): pass
  class StdQueries:
    TESTER_PRESENT_REQUEST = None
    UDS_VERSION_REQUEST = None
    TESTER_PRESENT_RESPONSE = None  
    UDS_VERSION_RESPONSE = None
    SUPPLIER_SOFTWARE_VERSION_REQUEST = None
    SUPPLIER_SOFTWARE_VERSION_RESPONSE = None
  HAS_FW_QUERY = False

Ecu = car.CarParams.Ecu

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

class CAR(Platforms):
  TESLA_PREAP_MODELS = PlatformConfig(
    [CarDocs("Tesla Pre-AP Model S", "All - 2013-2016 without Autopilot hardware")],
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=15.0),
    dbc_dict('tesla_can', 'tesla_radar', chassis_dbc='tesla_can')
  )
  TESLA_AP1_MODELS = PlatformConfig(
    [CarDocs("Tesla AP1 Model S", "All")],
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=15.0),
    dbc_dict('tesla_powertrain', 'tesla_radar_bosch_generated', chassis_dbc='tesla_can')
  )
  TESLA_AP2_MODELS = PlatformConfig(
    [CarDocs("Tesla AP2 Model S", "All")],
    TESLA_AP1_MODELS.specs,
    TESLA_AP1_MODELS.dbc_dict
  )
  TESLA_MODELS_RAVEN = PlatformConfig(
    [CarDocs("Tesla Model S Raven", "All")],
    TESLA_AP1_MODELS.specs,
    dbc_dict('tesla_powertrain', 'tesla_radar_continental_generated', chassis_dbc='tesla_can')
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.adas, Ecu.electricBrakeBooster, Ecu.fwdRadar],
      rx_offset=0x10,
      bus=0,
    ),
  ]
)

class CANBUS:
  # Lateral harness
  chassis = 0
  radar = 1
  autopilot_chassis = 2

  # Longitudinal harness
  powertrain = 4
  private = 5
  autopilot_powertrain = 6

GEAR_MAP = {
  "DI_GEAR_INVALID": car.CarState.GearShifter.unknown,
  "DI_GEAR_P": car.CarState.GearShifter.park,
  "DI_GEAR_R": car.CarState.GearShifter.reverse,
  "DI_GEAR_N": car.CarState.GearShifter.neutral,
  "DI_GEAR_D": car.CarState.GearShifter.drive,
  "DI_GEAR_SNA": car.CarState.GearShifter.unknown,
}

DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]

# Make sure the message and addr is also in the CAN parser!
BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [1]),
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [2]),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [4, 16]),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [8, 32]),
  Button(car.CarState.ButtonEvent.Type.cancel, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [1]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [2]),
]

class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7.0, 0.8])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8
  ACCEL_TO_SPEED_MULTIPLIER = 3

  def __init__(self, CP):
    pass


class CruiseButtons:
    # VAL_ 69 SpdCtrlLvr_Stat 32 "DN_1ST" 16 "UP_1ST" 8 "DN_2ND" 4 "UP_2ND" 2 "RWD" 1 "FWD" 0 "IDLE" ;
    RES_ACCEL = 16
    RES_ACCEL_2ND = 4
    DECEL_SET = 32
    DECEL_2ND = 8
    CANCEL = 1
    MAIN = 2
    IDLE = 0

    @classmethod
    def is_accel(cls, btn):
        return btn in [cls.RES_ACCEL, cls.RES_ACCEL_2ND]

    @classmethod
    def is_decel(cls, btn):
        return btn in [cls.DECEL_SET, cls.DECEL_2ND]

    @classmethod
    def should_be_throttled(cls, btn):
        # Some buttons should not be spammed or they may overwhelm the SCCM.
        return btn not in [cls.MAIN, cls.IDLE]

class CruiseState:
    # DI_cruiseState from the DBC
    OFF = 0
    STANDBY = 1
    ENABLED = 2
    STANDSTILL = 3
    OVERRIDE = 4
    FAULT = 5
    PRE_FAULT = 6
    PRE_CANCEL = 7

    @classmethod
    def is_enabled_or_standby(cls, state):
        return state in [cls.ENABLED, cls.STANDBY]

    @classmethod
    def is_faulted(cls, state):
        return state in [cls.PRE_FAULT, cls.FAULT]

    @classmethod
    def is_off(cls, state):
        return state in [cls.OFF]

# Additional Tesla constants
TESLA_MAX_ACCEL = 2.0  # m/s^2
TESLA_MIN_ACCEL = -3.5 # m/s^2
WHEEL_RADIUS = 0.353   # Tesla wheel radius

# CAN bus definitions for different Tesla models
CAN_CHASSIS = {
  CAR.TESLA_PREAP_MODELS: 0,
  CAR.TESLA_AP1_MODELS: 0,
  CAR.TESLA_AP2_MODELS: 0,
  CAR.TESLA_MODELS_RAVEN: 0,
}

CAN_POWERTRAIN = {
  CAR.TESLA_PREAP_MODELS: -1,  # Pre-AP models don't have powertrain bus
  CAR.TESLA_AP1_MODELS: 0,
  CAR.TESLA_AP2_MODELS: 4,
  CAR.TESLA_MODELS_RAVEN: 4,
}

CAN_AUTOPILOT = {
  CAR.TESLA_PREAP_MODELS: -1,  # Pre-AP models don't have autopilot bus
  CAR.TESLA_AP1_MODELS: 2,
  CAR.TESLA_AP2_MODELS: 2,
  CAR.TESLA_MODELS_RAVEN: 2,
}

CAN_EPAS = {
  CAR.TESLA_PREAP_MODELS: 0,
  CAR.TESLA_AP1_MODELS: 0,
  CAR.TESLA_AP2_MODELS: 0,
  CAR.TESLA_MODELS_RAVEN: 0,
}

DBC = CAR.create_dbc_map()
