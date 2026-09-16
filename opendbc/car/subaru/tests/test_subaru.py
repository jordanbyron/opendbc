import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC as CANDBC
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carcontroller import CarController, SNG_ACC_MAX_DIST, SNG_ACC_MIN_DIST, SNG_HOLD_FRAMES, SNG_RESUME_FRAMES
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, DBC, CanBus, CarControllerParams, SubaruFlags, SubaruSafetyFlags


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruImprezaTorque(unittest.TestCase):
  def _cp(self, car, on):
    return CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False, increased_steer_torque=on)

  def test_off_is_stock(self):
    CP = self._cp(CAR.SUBARU_IMPREZA, False)
    assert not CP.flags & SubaruFlags.IMPREZA_TORQUE
    assert not CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.IMPREZA_TORQUE
    assert CarControllerParams(CP).STEER_MAX == 2047
    self.assertAlmostEqual(CP.lateralTuning.pid.kf, 0.00005)
    for got, exp in zip(CP.lateralTuning.pid.kpV, [0.2, 0.3], strict=True):
      self.assertAlmostEqual(got, exp)
    for got, exp in zip(CP.lateralTuning.pid.kiV, [0.02, 0.03], strict=True):
      self.assertAlmostEqual(got, exp)

  def test_on_impreza(self):
    CP = self._cp(CAR.SUBARU_IMPREZA, True)
    assert CP.flags & SubaruFlags.IMPREZA_TORQUE
    assert CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.IMPREZA_TORQUE
    assert CarControllerParams(CP).STEER_MAX == 3071
    self.assertAlmostEqual(CP.lateralTuning.pid.kf, 0.00005 / 1.5)
    for got, exp in zip(CP.lateralTuning.pid.kpV, [0.2 / 1.5, 0.3 / 1.5], strict=True):
      self.assertAlmostEqual(got, exp)
    for got, exp in zip(CP.lateralTuning.pid.kiV, [0.02 / 1.5, 0.03 / 1.5], strict=True):
      self.assertAlmostEqual(got, exp)

  def test_ignored_on_other_platforms(self):
    for car in CAR:
      if car == CAR.SUBARU_IMPREZA:
        continue
      off, on = self._cp(car, False), self._cp(car, True)
      assert not on.flags & SubaruFlags.IMPREZA_TORQUE
      assert on.safetyConfigs[0].safetyParam == off.safetyConfigs[0].safetyParam
      assert CarControllerParams(on).STEER_MAX == CarControllerParams(off).STEER_MAX


class TestSubaruStopAndGo(unittest.TestCase):
  def _cp(self, car, alpha_long=False):
    return CarInterface.get_params(car, gen_empty_fingerprint(), [], alpha_long=alpha_long, is_release=False, docs=False)

  def test_eligible_impreza(self):
    CP = self._cp(CAR.SUBARU_IMPREZA)
    assert CarInterface.enable_stop_and_go(CP)
    assert CP.flags & SubaruFlags.SNG
    assert CP.safetyConfigs[0].safetyParam & SubaruSafetyFlags.SNG

  def test_ineligible_platforms(self):
    for car, alpha_long in ((CAR.SUBARU_OUTBACK, False), (CAR.SUBARU_CROSSTREK_HYBRID, False),
                            (CAR.SUBARU_FORESTER_PREGLOBAL, False), (CAR.SUBARU_IMPREZA, True)):
      with self.subTest(car=car, alpha_long=alpha_long):
        CP = self._cp(car, alpha_long=alpha_long)
        before = CP.to_bytes()
        CP.clear_write_flag()
        assert not CarInterface.enable_stop_and_go(CP)
        assert CP.to_bytes() == before

  def test_packer_round_trip(self):
    packer = CANPacker(DBC[CAR.SUBARU_IMPREZA][Bus.pt])
    parser = CANParser(DBC[CAR.SUBARU_IMPREZA][Bus.pt], [("Throttle", 100), ("Brake_Pedal", 50)], CanBus.camera)

    throttle_msg = {"CHECKSUM": 0, "COUNTER": 0, "Signal1": 3, "Engine_RPM": 1234, "Neutral": 1,
                    "Throttle_Pedal": 40, "Throttle_Cruise": 41, "Throttle_Combo": 42, "Signal3": 5, "Off_Accel": 6}
    brake_msg = {"CHECKSUM": 0, "COUNTER": 0, "Signal1": 3, "Speed": 11.98125, "Signal2": 7,
                 "Brake_Lights": 0, "Signal3": 1, "Brake_Pedal": 700, "Signal4": 4242}

    def decode(msg):
      parser.update([0, [msg]])
      return parser

    decode(subarucan.create_throttle(packer, 3, throttle_msg, False))
    for sig, val in throttle_msg.items():
      if sig not in ("CHECKSUM", "COUNTER"):
        self.assertAlmostEqual(parser.vl["Throttle"][sig], val, msg=sig)
    self.assertEqual(parser.vl["Throttle"]["COUNTER"], 3)

    decode(subarucan.create_brake_pedal(packer, 19, brake_msg, False, False))
    for sig, val in brake_msg.items():
      if sig not in ("CHECKSUM", "COUNTER"):
        self.assertAlmostEqual(parser.vl["Brake_Pedal"][sig], val, msg=sig)
    self.assertEqual(parser.vl["Brake_Pedal"]["COUNTER"], 19 % 0x10)

    # FrogPilot parity: the spoof values replace whatever the car reported, they are not floors
    decode(subarucan.create_throttle(packer, 0, dict(throttle_msg, Throttle_Pedal=0), True))
    self.assertEqual(parser.vl["Throttle"]["Throttle_Pedal"], subarucan.SNG_THROTTLE_SPOOF)
    decode(subarucan.create_throttle(packer, 0, throttle_msg, True))
    self.assertEqual(parser.vl["Throttle"]["Throttle_Pedal"], subarucan.SNG_THROTTLE_SPOOF)

    decode(subarucan.create_brake_pedal(packer, 0, brake_msg, True, True))
    self.assertEqual(parser.vl["Brake_Pedal"]["Brake_Pedal"], subarucan.SNG_BRAKE_SPOOF)
    self.assertEqual(parser.vl["Brake_Pedal"]["Brake_Lights"], 1)
    # Speed is a 0.05625 kph/bit field; FrogPilot writes 3 (km/h, physical units), i.e. 53 raw counts
    self.assertAlmostEqual(parser.vl["Brake_Pedal"]["Speed"], subarucan.SNG_SPEED_SPOOF, places=1)
    self.assertEqual(subarucan.SNG_SPEED_SPOOF, 3)

    # with neither command the car's own values pass straight through
    decode(subarucan.create_brake_pedal(packer, 0, brake_msg, False, False))
    self.assertEqual(parser.vl["Brake_Pedal"]["Brake_Pedal"], 700)
    self.assertAlmostEqual(parser.vl["Brake_Pedal"]["Speed"], 11.98125)


class StubCarState:
  def __init__(self, dbc_name):
    dbc = CANDBC(dbc_name)
    self.out = structs.CarState()
    self.car_follow = 0
    self.cruise_state = 0
    self.close_distance = 0.
    for attr, msg in (("throttle_msg", "Throttle"), ("brake_pedal_msg", "Brake_Pedal"),
                      ("es_dashstatus_msg", "ES_DashStatus"), ("es_lkas_state_msg", "ES_LKAS_State"),
                      ("es_distance_msg", "ES_Distance")):
      setattr(self, attr, {s: 0 for s in dbc.name_to_msg[msg].sigs})


class TestSubaruStopAndGoController(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.SUBARU_IMPREZA, gen_empty_fingerprint(), [], alpha_long=False,
                                      is_release=False, docs=False)
    CarInterface.enable_stop_and_go(self.CP)
    self.CC = CarController(DBC[self.CP.carFingerprint], self.CP)
    self.CS = StubCarState(DBC[self.CP.carFingerprint][Bus.pt])
    cc = structs.CarControl()
    cc.enabled = True
    self.cc = cc.as_reader()

  def _lead_pulls_away(self, close_distance):
    self.CS.car_follow = 1
    self.CS.cruise_state = 3
    self.CS.out.standstill = True
    self.CS.close_distance = close_distance

  def test_resume_pulse_length(self):
    # FrogPilot's state machine: the first pulse is SNG_RESUME_FRAMES long, then the counter rests at -1
    # so every later pulse is one frame longer. A re-trigger mid-pulse does not extend it.
    self._lead_pulls_away(3.5)
    for i in range(8):
      throttle_cmd, _ = self.CC.stop_and_go(self.cc, self.CS)
      self.assertEqual(throttle_cmd, i < SNG_RESUME_FRAMES, i)
    self.assertEqual(self.CC.sng_acc_resume_cnt, -1)

    # second trigger: close_distance rises again inside the window
    self.CS.close_distance = 3.0
    self.CC.stop_and_go(self.cc, self.CS)
    self.CS.close_distance = 3.5
    pulses = []
    for _ in range(8):
      throttle_cmd, _ = self.CC.stop_and_go(self.cc, self.CS)
      pulses.append(throttle_cmd)
    self.assertEqual(sum(pulses), SNG_RESUME_FRAMES + 1)

  def test_resume_pulse_ignores_driver_gas(self):
    # FrogPilot does not mask the pulse on a driver gas press
    self._lead_pulls_away(3.5)
    self.CS.out.gasPressed = True
    throttle_cmd, _ = self.CC.stop_and_go(self.cc, self.CS)
    self.assertTrue(throttle_cmd)

  def test_no_resume_outside_distance_window(self):
    for close_distance in (SNG_ACC_MIN_DIST, SNG_ACC_MAX_DIST):
      with self.subTest(close_distance=close_distance):
        CC = CarController(DBC[self.CP.carFingerprint], self.CP)
        self._lead_pulls_away(close_distance)
        throttle_cmd, _ = CC.stop_and_go(self.cc, self.CS)
        self.assertFalse(throttle_cmd)

  def test_manual_hold_suppresses_resume(self):
    # driver braked to a stop with no lead: Cruise_State 1 -> 3 while stopped
    self.CS.out.standstill = True
    self.CS.cruise_state = 1
    self.CC.stop_and_go(self.cc, self.CS)
    self.CS.cruise_state = 3
    self.CC.stop_and_go(self.cc, self.CS)
    self.assertTrue(self.CC.manual_hold)

    self._lead_pulls_away(3.5)
    throttle_cmd, _ = self.CC.stop_and_go(self.cc, self.CS)
    self.assertFalse(throttle_cmd)

    # rolling again clears the hold
    self.CS.out.standstill = False
    self.CC.stop_and_go(self.cc, self.CS)
    self.assertFalse(self.CC.manual_hold)

  def test_speed_spoof_waits_out_the_hold(self):
    # roll first so standstill_start is latched on a real rising edge
    self.CS.car_follow = 1
    self.CS.out.standstill = False
    self.CC.frame = 100
    self.CC.stop_and_go(self.cc, self.CS)
    self.CS.out.standstill = True
    self.CS.close_distance = 0.
    # FrogPilot evaluates the hold timer before latching the edge, so the very first stopped frame may
    # emit speed_cmd against the stale standstill_start (here 0, so frame 100 > 0 + 50). Parity quirk.
    _, speed_cmd = self.CC.stop_and_go(self.cc, self.CS)
    self.assertTrue(speed_cmd)
    self.CC.frame += 1
    for _ in range(SNG_HOLD_FRAMES):
      _, speed_cmd = self.CC.stop_and_go(self.cc, self.CS)
      self.assertFalse(speed_cmd)
      self.CC.frame += 1
    _, speed_cmd = self.CC.stop_and_go(self.cc, self.CS)
    self.assertTrue(speed_cmd)

  def _sent_addrs(self, frames):
    sent = []
    for _ in range(frames):
      _, can_sends = self.CC.update(self.cc, self.CS, 0)
      sent.append([(addr, bus, dat) for addr, dat, bus in can_sends])
    return sent

  def test_update_emits_mirror(self):
    for frame, msgs in enumerate(self._sent_addrs(10)):
      addrs = [addr for addr, bus, _ in msgs if bus == CanBus.camera]
      self.assertIn(0x40, addrs)
      self.assertEqual(0x139 in addrs, frame % 2 == 0)
      for addr, _, dat in msgs:
        if addr == 0x40:
          # re-based off the last stock frame's counter, as in FrogPilot
          self.assertEqual(dat[1] & 0xF, (self.CS.throttle_msg["COUNTER"] + 1) % 0x10)
        elif addr == 0x139:
          self.assertEqual(dat[1] & 0xF, (frame // 2) % 0x10)

  def test_no_mirror_without_flag(self):
    self.CP.flags = self.CP.flags & ~int(SubaruFlags.SNG)
    self.CC = CarController(DBC[self.CP.carFingerprint], self.CP)
    for msgs in self._sent_addrs(10):
      for addr, _, _ in msgs:
        self.assertNotIn(addr, (0x40, 0x139))
