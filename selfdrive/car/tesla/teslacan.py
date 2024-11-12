import crcmod

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.tesla.values import CANBUS, CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

    self.min_accel_last = 0
    self.max_accel_last = 0

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[2]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, active):
    self.max_accel_last = max(accel, 0)
    self.min_accel_last = accel
    values = {
      "DAS_setSpeed": 0 if (accel < 0 or not active) else V_CRUISE_MAX,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": self.min_accel_last,
      "DAS_accelMax": self.max_accel_last,
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def stock_longitudinal(self, acc_state, accel, das_control, cntr, active, speed):
    speed = speed * CV.MS_TO_KPH

    # accelMax
    if speed > 40 and (das_control["DAS_accelMax"] >= accel >= das_control["DAS_accelMin"]):
      max_accel = accel
    else:
      max_accel = das_control["DAS_accelMax"]
    max_tr = 0.03 if speed < 40 else 0.005
    max_accel = clip(max_accel, self.max_accel_last - max_tr, self.max_accel_last + max_tr)
    self.max_accel_last = clip(max(max_accel, 0), -3.48, 2)

    # accelMin
    min_accel = clip(das_control["DAS_accelMin"], -3.48, 2)
    if min_accel < 0 and accel >= 0:
      min_tr = 0.001
    elif min_accel < -1 and accel < -1:
      min_tr = 0.04
    else:
      min_tr = 0.01
    min_accel = clip(min_accel, self.min_accel_last - min_tr, self.min_accel_last + min_tr)
    self.min_accel_last = min_accel

    values = {
      "DAS_setSpeed": 0 if not active else das_control["DAS_setSpeed"],
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": das_control["DAS_jerkMin"],
      "DAS_jerkMax": das_control["DAS_jerkMax"],
      "DAS_accelMin": self.min_accel_last,
      "DAS_accelMax": self.max_accel_last,
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

