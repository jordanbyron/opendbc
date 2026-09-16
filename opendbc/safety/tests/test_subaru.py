#!/usr/bin/env python3
import enum
import unittest
from collections import defaultdict

from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from functools import partial


class SubaruMsg(enum.IntEnum):
  Brake_Pedal       = 0x139
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Brake          = 0x220
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323
  ES_UDS_Request    = 0x787
  ES_HighBeamAssist = 0x22A
  ES_STATIC_1       = 0x325
  ES_STATIC_2       = 0x121


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2

RX_WINDOW = 6              # enough frames for every rx check to see a valid counter sequence
SPEED_TO_KPH = 0.05625     # Brake_Pedal->Speed, kph per count
SNG_THROTTLE_SPOOF = 5     # FrogPilot's spoof values, in raw counts
SNG_BRAKE_SPOOF = 5
SNG_SPEED_SPOOF_KPH = 3


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS):
  return [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def long_tx_msgs(alt_bus):
  return [[SubaruMsg.ES_Brake,          alt_bus],
          [SubaruMsg.ES_Status,         alt_bus]]


def gen2_long_additional_tx_msgs():
  return [[SubaruMsg.ES_UDS_Request,    SUBARU_CAM_BUS],
          [SubaruMsg.ES_HighBeamAssist, SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_1,       SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_2,       SUBARU_MAIN_BUS]]


def sng_tx_msgs():
  return [[SubaruMsg.Throttle,    SUBARU_CAM_BUS],
          [SubaruMsg.Brake_Pedal, SUBARU_CAM_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS):
  return {SUBARU_CAM_BUS: [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)

  def _set_aol_acc_main(self, main_on: bool):
    values = {"Cruise_On": main_on}
    self._rx(self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values))

  def _aol_invalid_rx_msg(self):
    def corrupt_checksum(msg):
      addr, dat, bus = msg
      return addr, bytes([dat[0] ^ 0xFF]) + dat[1:], bus

    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, {"Cruise_On": True},
                                           fix_checksum=corrupt_checksum)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruLongitudinalSafetyBase(TestSubaruSafetyBase, common.LongitudinalGasBrakeSafetyTest):
  MIN_GAS = 808
  MAX_GAS = 3400
  INACTIVE_GAS = 1818
  MAX_POSSIBLE_GAS = 2**13

  MIN_BRAKE = 0
  MAX_BRAKE = 600
  MAX_POSSIBLE_BRAKE = 2**16

  MIN_RPM = 0
  MAX_RPM = 3600
  MAX_POSSIBLE_RPM = 2**13

  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance,
                               SubaruMsg.ES_Status, SubaruMsg.ES_DashStatus,
                               SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}

  def test_rpm_safety_check(self):
    self._generic_limit_safety_check(self._send_rpm_msg, self.MIN_RPM, self.MAX_RPM, 0, self.MAX_POSSIBLE_RPM, 1)

  def _send_brake_msg(self, brake):
    values = {"Brake_Pressure": brake}
    return self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"Cruise_Throttle": gas}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def _send_rpm_msg(self, rpm):
    values = {"Cruise_RPM": rpm}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.AlwaysOnLateralTorqueSteeringSafetyTest,
                                 common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)

  def test_stock_gen1_cap_boundary(self):
    # deltas from _set_prev_torque are 0 or 1, so rate and RT-delta limits never fire here
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(2047)
    self.assertEqual(self._tx(self._torque_cmd_msg(2048)), self.MAX_TORQUE >= 2048)
    self._set_prev_torque(3071)
    self.assertEqual(self._tx(self._torque_cmd_msg(3071)), self.MAX_TORQUE >= 3071)
    self.assertFalse(self._tx(self._torque_cmd_msg(3072)))


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruGen1LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS) + long_tx_msgs(SUBARU_MAIN_BUS)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Status,
                                               SubaruMsg.ES_Distance)}


class TestSubaruGen2LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS) + long_tx_msgs(SUBARU_ALT_BUS) + gen2_long_additional_tx_msgs()
  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                               SubaruMsg.ES_Infotainment]}
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment),
                             SUBARU_ALT_BUS: (SubaruMsg.ES_Brake, SubaruMsg.ES_Status, SubaruMsg.ES_Distance)}

  def _rdbi_msg(self, did: int):
    return b'\x03\x22' + did.to_bytes(2) + b'\x00\x00\x00\x00'

  def _es_uds_msg(self, msg: bytes):
    return libsafety_py.make_CANPacket(SubaruMsg.ES_UDS_Request, 2, msg)

  def test_es_uds_message(self):
    tester_present = b'\x02\x3E\x80\x00\x00\x00\x00\x00'
    not_tester_present = b"\x03\xAA\xAA\x00\x00\x00\x00\x00"

    button_did = 0x1130

    # Tester present is allowed for gen2 long to keep eyesight disabled
    self.assertTrue(self._tx(self._es_uds_msg(tester_present)))

    # Non-Tester present is not allowed
    self.assertFalse(self._tx(self._es_uds_msg(not_tester_present)))

    # Only button_did is allowed to be read via UDS
    for did in range(0xFFFF):
      should_tx = (did == button_did)
      self.assertEqual(self._tx(self._es_uds_msg(self._rdbi_msg(did))), should_tx)

    # any other msg is not allowed
    for sid in range(0xFF):
      msg = b'\x03' + sid.to_bytes(1) + b'\x00' * 6
      self.assertFalse(self._tx(self._es_uds_msg(msg)))


class TestSubaruGen1ImprezaTorqueStockLongitudinalSafety(TestSubaruGen1TorqueStockLongitudinalSafety):
  FLAGS = SubaruSafetyFlags.IMPREZA_TORQUE
  MAX_TORQUE_LOOKUP = [0], [3071]


class TestSubaruGen1ImprezaTorqueLongitudinalSafety(TestSubaruGen1LongitudinalSafety):
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.IMPREZA_TORQUE
  MAX_TORQUE_LOOKUP = [0], [3071]


class TestSubaruGen2ImprezaTorqueIgnoredSafety(TestSubaruGen2TorqueStockLongitudinalSafety):
  # the flag is gen1-only; gen2 keeps its 1000 cap
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.IMPREZA_TORQUE


class TestSubaruGen1TorqueStockLongitudinalSngSafety(TestSubaruGen1TorqueStockLongitudinalSafety):
  # Stop and Go, as in FrogPilot: openpilot re-sends Throttle (0x40) and Brake_Pedal (0x139) on the camera
  # bus and panda blocks the stock copies main->cam. panda does not bound the mirrored fields and does not
  # add an rx check for Brake_Pedal.
  FLAGS = SubaruSafetyFlags.SNG
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS) + sng_tx_msgs()
  FWD_BLACKLISTED_ADDRS = {**fwd_blacklisted_addr(), SUBARU_MAIN_BUS: [SubaruMsg.Throttle, SubaruMsg.Brake_Pedal]}

  def setUp(self):
    super().setUp()
    self._rx_counters: dict[str, int] = defaultdict(int)

  def _rx_window(self, name, values, bus=SUBARU_MAIN_BUS):
    # counters have to advance or the rx check invalidates the message part way through
    for _ in range(RX_WINDOW):
      self._rx_counters[name] = (self._rx_counters[name] + 1) % 16
      msg = self.packer.make_can_msg_safety(name, bus, {**values, "COUNTER": self._rx_counters[name]})
      self.assertTrue(self._rx(msg))

  def _cam_throttle_msg(self, pedal):
    return self.packer.make_can_msg_safety("Throttle", SUBARU_CAM_BUS, {"Throttle_Pedal": pedal, "COUNTER": 0})

  def _cam_brake_pedal_msg(self, brake=0, speed_kph=0, lights=0):
    values = {"Brake_Pedal": brake, "Speed": speed_kph, "Brake_Lights": lights, "COUNTER": 0}
    return self.packer.make_can_msg_safety("Brake_Pedal", SUBARU_CAM_BUS, values)

  def test_sng_mirrors_unbounded(self):
    # FrogPilot parity: the mirrors are allowed regardless of engagement and regardless of field values,
    # so EyeSight is never starved of either message while the stock copies are blocked
    for controls_allowed in (False, True):
      with self.subTest(controls_allowed=controls_allowed):
        self.safety.set_controls_allowed(controls_allowed)
        for pedal in (0, SNG_THROTTLE_SPOOF, 255):
          self.assertTrue(self._tx(self._cam_throttle_msg(pedal)))
        for brake, speed in ((0, 0), (SNG_BRAKE_SPOOF, SNG_SPEED_SPOOF_KPH), (4095, 200)):
          self.assertTrue(self._tx(self._cam_brake_pedal_msg(brake=brake, speed_kph=speed, lights=1)))

  def test_sng_stock_copies_pass_on_cam_bus_without_relay_fault(self):
    # a stock 0x40/0x139 seen on the camera bus must not be treated as a relay malfunction (FrogPilot
    # blocks by fwd hook only; nothing arms stock_ecu_check for these)
    self._rx(self._cam_throttle_msg(0))
    self._rx(self._cam_brake_pedal_msg())
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_sng_no_brake_pedal_rx_check(self):
    # the rx checks are the stock five: the config is valid without ever seeing Brake_Pedal
    self._reset_safety_hooks()
    self._rx_counters.clear()
    for name, values in (("Throttle", {"Throttle_Pedal": 0}),
                         ("Steering_Torque", {"Steer_Torque_Sensor": 0}),
                         ("Wheel_Speeds", {s: 0 for s in ("FR", "FL", "RR", "RL")}),
                         ("Brake_Status", {"Brake": 0}),
                         ("CruiseControl", {"Cruise_On": 1})):
      self._rx_window(name, values)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_sng_throttle_rx_not_user_gas(self):
    # our own copy on the camera bus must never read back as a driver gas press
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self._cam_throttle_msg(SNG_THROTTLE_SPOOF))
    self.assertFalse(self.safety.get_gas_pressed_prev())


class TestSubaruGen1ImprezaTorqueSngSafety(TestSubaruGen1TorqueStockLongitudinalSngSafety):
  # the only config matching the device with all three features on; no single feature's plan covers it
  FLAGS = SubaruSafetyFlags.IMPREZA_TORQUE | SubaruSafetyFlags.SNG
  MAX_TORQUE_LOOKUP = [0], [3071]

  def test_aol_keeps_impreza_cap_boundary(self):
    # deltas from _set_prev_torque are 0 or 1, so rate and RT-delta limits never fire here
    self._prepare_aol_lateral()
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(2047)
    self.assertTrue(self._tx(self._torque_cmd_msg(2048)))
    self._set_prev_torque(3071)
    self.assertTrue(self._tx(self._torque_cmd_msg(3071)))
    self._set_prev_torque(3071)
    self.assertFalse(self._tx(self._torque_cmd_msg(3072)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_aol_independent_of_brake_pedal_rx(self):
    # SNG adds no rx check, so always-on lateral is unaffected by whether Brake_Pedal was ever seen
    self._reset_safety_hooks()
    self._rx_counters.clear()
    self._set_aol_experience(True)
    for name, values in (("Throttle", {"Throttle_Pedal": 0}),
                         ("Steering_Torque", {"Steer_Torque_Sensor": 0}),
                         ("Wheel_Speeds", {s: 0 for s in ("FR", "FL", "RR", "RL")}),
                         ("Brake_Status", {"Brake": 0}),
                         ("CruiseControl", {"Cruise_On": 1})):
      self._rx_window(name, values)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())
    self._set_aol_acc_main(True)
    self.assertTrue(self.safety.get_aol_allowed())
    self._assert_aol_lateral(True)


class TestSubaruGen2TorqueStockLongitudinalSngIgnoredSafety(TestSubaruGen2TorqueStockLongitudinalSafety):
  # SNG is gen1-only; the flag must not change the TX list or the forwarding
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.SNG


class TestSubaruGen1LongitudinalSngIgnoredSafety(TestSubaruGen1LongitudinalSafety):
  # SNG is stock-longitudinal only
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.SNG


if __name__ == "__main__":
  unittest.main()
