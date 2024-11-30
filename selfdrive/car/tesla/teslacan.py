import time

import crcmod

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.tesla.values import CANBUS, CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
    self.max_tacc_time = time.time()
    self.max_op_time = time.time()
    self.min_tacc_time = time.time()
    self.min_op_time = time.time()

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
    values = {
      "DAS_setSpeed": 0 if (accel < 0 or not active) else V_CRUISE_MAX,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def hybrid_longitudinal(self, acc_state, accel, das_control, cntr, speed, gas_pressed):
    speed = speed * CV.MS_TO_KPH

    # Improve behavior during stop-and-go traffic
    if speed <= 25 or gas_pressed:
      max_accel = das_control["DAS_accelMax"]
      self.max_tacc_time = time.time()
    else:
      # Blending from stock ACC to openpilot longitudinal
      max_fade = 2.0
      factor = max(time.time() - self.max_tacc_time, max_fade) / max_fade
      max_accel = (1 - factor) * das_control["DAS_accelMax"] + factor * max(accel, 0.4)

    if (-0.5 > accel > das_control["DAS_accelMin"]) or gas_pressed:
      min_fade = 1.0  # 1 second
      factor = max(time.time() - self.min_op_time, min_fade) / min_fade
      min_accel = (1 - factor) * das_control["DAS_accelMin"] + factor * accel
      self.min_tacc_time = time.time()
    else:
      min_fade = 1.0  # 1 second
      factor = max(time.time() - self.min_tacc_time, min_fade) / min_fade
      min_accel = (1 - factor) * accel + factor * das_control["DAS_accelMin"]
      self.min_op_time = time.time()

    max_accel = clip(max_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    min_accel = clip(min_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    values = {
      "DAS_setSpeed": das_control["DAS_setSpeed"],
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": das_control["DAS_jerkMin"],
      "DAS_jerkMax": das_control["DAS_jerkMax"],
      "DAS_accelMin": min(min_accel, -0.4),
      "DAS_accelMax": max(max_accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)
