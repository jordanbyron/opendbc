import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruFlags, SubaruSafetyFlags


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
