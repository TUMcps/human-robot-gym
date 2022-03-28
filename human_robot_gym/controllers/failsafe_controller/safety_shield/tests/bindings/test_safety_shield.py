import os
import pytest
from safety_shield_py import Motion
from safety_shield_py import SafetyShield

class TestSafetyShield:
  @pytest.fixture
  def shield(self):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)

    shield = SafetyShield(
     activate_shield = True,
     sample_time = 0.001,
     trajectory_config_file = dir_path + "/../../config/trajectory_parameters_modrob1.yaml",
     robot_config_file = dir_path + "/../../config/robot_parameters_modrob1.yaml",
     mocap_config_file = dir_path + "/../../config/cmu_mocap_no_hand.yaml",
     init_x = 0.0,
     init_y = 0.0,
     init_z = 0.0,
     init_roll = 0.0,
     init_pitch = 0.0,
     init_yaw = 0.0
    )
    return shield

  def test_safety_shield(self, shield):
    assert(shield is not None)
  
  def test_humanMeasurement(self, shield):
    dummy_meas = []
    for i in range(21):
      dummy_meas.append([0.0, 0.0, 0.0])
    shield.humanMeasurement(dummy_meas, 0.0)
  
  def test_newLTT(self, shield):
    motion = Motion(6)
    shield.newLongTermTrajectory(motion)

  def test_step(self, shield):
    shield.step(0.004)
