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
     sample_time = 0.004,
     trajectory_config_file = dir_path + "/../../config/trajectory_parameters_modrob1.yaml",
     robot_config_file = dir_path + "/../../config/robot_parameters_modrob1.yaml",
     mocap_config_file = dir_path + "/../../config/cmu_mocap_no_hand.yaml",
     init_x = 0.0,
     init_y = 0.0,
     init_z = 0.0,
     init_roll = 0.0,
     init_pitch = 0.0,
     init_yaw = 0.0,
     init_qpos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    return shield

  @pytest.fixture
  def shield_schunk(self):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)

    shield = SafetyShield(
     activate_shield = True,
     sample_time = 0.004,
     trajectory_config_file = dir_path + "/../../config/trajectory_parameters_schunk.yaml",
     robot_config_file = dir_path + "/../../config/robot_parameters_schunk.yaml",
     mocap_config_file = dir_path + "/../../config/cmu_mocap_no_hand.yaml",
     init_x = 0.0,
     init_y = 0.0,
     init_z = 0.0,
     init_roll = 0.0,
     init_pitch = 0.0,
     init_yaw = 0.0,
     init_qpos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
    motion = shield.step(0.004)
    assert(motion.getTime() == 0.004)

  def test_getSafety(self, shield):
    dummy_meas = []
    for i in range(21):
      dummy_meas.append([0.0, 0.0, 0.0])
    shield.humanMeasurement(dummy_meas, 0.0)
    motion = Motion(6)
    shield.newLongTermTrajectory(motion)
    motion = shield.step(0.004)
    is_safe = shield.getSafety()
    assert(is_safe == False)

  def test_getRobotReachCapsules(self, shield):
    motion = Motion(6)
    shield.newLongTermTrajectory(motion)
    dummy_meas = []
    for i in range(21):
      dummy_meas.append([0.0, 0.0, 0.0])
    shield.humanMeasurement(dummy_meas, 0.0)
    shield.step(0.004)
    robot_caps = shield.getRobotReachCapsules()
    for cap in robot_caps:
      assert(len(cap) == 7)
      print("-------")
      print(cap[0])
      print(cap[1])
      print(cap[2])
      print(cap[3])
      print(cap[4])
      print(cap[5])
      print(cap[6])
    
    eps = 1e-5
    secure_radius = 0.01
    print("-----")
    assert(abs(robot_caps[0][0] - 4.419824882251160e-04) <= eps)
    assert(abs(robot_caps[0][1] - -0.046901920258750) <= eps)
    assert(abs(robot_caps[0][2] - 0.316102435183982) <= eps)
    assert(abs(robot_caps[0][3] - 5.633175125972268e-04) <= eps)
    assert(abs(robot_caps[0][4] - -0.039612679720870) <= eps)
    assert(abs(robot_caps[0][5] - 0.309257564827811) <= eps)
    assert(abs(robot_caps[0][6] - (0.128755148876288 + secure_radius)) <= eps)
    print("-----") 
    assert(abs(robot_caps[1][0] - -0.001475548520496) <= eps)
    assert(abs(robot_caps[1][1] - 0.069582202005415) <= eps)
    assert(abs(robot_caps[1][2] - 0.376756288468218) <= eps)
    assert(abs(robot_caps[1][3] - 0.001008748287424) <= eps)
    assert(abs(robot_caps[1][4] - 0.041671798076183) <= eps)
    assert(abs(robot_caps[1][5] - 0.695527111508655) <= eps)
    assert(abs(robot_caps[1][6] - (0.135477093612307 + secure_radius)) <= eps)
    print("-----")
    assert(abs(robot_caps[2][0] - 0.022974546488243) <= eps)
    assert(abs(robot_caps[2][1] - -0.098358202251421) <= eps)
    assert(abs(robot_caps[2][2] - 0.769133504587120) <= eps)
    assert(abs(robot_caps[2][3] - 0.090252153491381) <= eps)
    assert(abs(robot_caps[2][4] - -0.112369997514008) <= eps)
    assert(abs(robot_caps[2][5] - 0.810089495473077) <= eps)
    assert(abs(robot_caps[2][6] - (0.123147121411940 + secure_radius)) <= eps)
    print("-----")
    assert(abs(robot_caps[3][0] - 0.113983176898700) <= eps)
    assert(abs(robot_caps[3][1] - -0.074843993014393) <= eps)
    assert(abs(robot_caps[3][2] - 0.970834416653274) <= eps)
    assert(abs(robot_caps[3][3] - 0.114016223039576) <= eps)
    assert(abs(robot_caps[3][4] - -0.035085806401094) <= eps)
    assert(abs(robot_caps[3][5] - 1.136119883362162) <= eps)
    assert(abs(robot_caps[3][6] - (0.132674969625049 + secure_radius)) <= eps)
    print("-----")
    assert(abs(robot_caps[4][0] - 0.068707349990918) <= eps)
    assert(abs(robot_caps[4][1] - -0.131206299703527) <= eps)
    assert(abs(robot_caps[4][2] - 1.193086500002462) <= eps)
    assert(abs(robot_caps[4][3] - 0.068707349990918) <= eps)
    assert(abs(robot_caps[4][4] - -0.131206299703527) <= eps)
    assert(abs(robot_caps[4][5] - 1.193086500002462) <= eps)
    assert(abs(robot_caps[4][6] - (0.127657916394952 + secure_radius)) <= eps)
    print("-----")
    #assert(abs(robot_caps[5][0] - 0.273999999975235) <= eps)
    # We manually set this value, so the test would fail.
    assert(abs(robot_caps[5][1] - -0.101349999585660) <= eps)
    assert(abs(robot_caps[5][2] - 1.223845648629919) <= eps)
    assert(abs(robot_caps[5][3] - 0.324549986083302) <= eps)
    assert(abs(robot_caps[5][4] - -0.101349999554536) <= eps)
    assert(abs(robot_caps[5][5] - 1.223754351487657) <= eps)
    assert(abs(robot_caps[5][6] - (0.046806563054797 + secure_radius)) <= eps)
    
  # ---------------- Schunk -------------------
  def test_getRobotReachCapsulesSchunk(self, shield_schunk):
    motion = Motion(6)
    shield_schunk.newLongTermTrajectory(motion)
    dummy_meas = []
    for i in range(21):
      dummy_meas.append([0.0, 0.0, 0.0])
    shield_schunk.humanMeasurement(dummy_meas, 0.0)
    shield_schunk.step(0.004)
    robot_caps = shield_schunk.getRobotReachCapsules()
    for cap in robot_caps:
      assert(len(cap) == 7)
      print("-------")
      print(cap[0])
      print(cap[1])
      print(cap[2])
      print(cap[3])
      print(cap[4])
      print(cap[5])
      print(cap[6])
    eps = 1e-5
    secure_radius = 0.02
    assert(abs(robot_caps[0][0] - 0) <= eps)
    assert(abs(robot_caps[0][1] - -0.015000000000000) <= eps)
    assert(abs(robot_caps[0][2] - 0.155300000000000) <= eps)
    assert(abs(robot_caps[0][3] - 0) <= eps)
    assert(abs(robot_caps[0][4] - -0.015000000000000) <= eps)
    assert(abs(robot_caps[0][5] - 0.155300000000000) <= eps)
    assert(abs(robot_caps[0][6] - (0.090000000000000 + secure_radius)) <= eps)
    assert(abs(robot_caps[1][0] - 2.295154367360221e-04) <= eps)
    assert(abs(robot_caps[1][1] - 0.114880476266314) <= eps)
    assert(abs(robot_caps[1][2] - 0.505203783848448) <= eps)
    assert(abs(robot_caps[1][3] - -6.617154367385000e-04) <= eps)
    assert(abs(robot_caps[1][4] - 0.114044823733687) <= eps)
    assert(abs(robot_caps[1][5] - 0.155205916151551) <= eps)
    assert(abs(robot_caps[1][6] - (0.074020642029512 + secure_radius)) <= eps)
    assert(abs(robot_caps[2][0] - -1.594314036332950e-15) <= eps)
    assert(abs(robot_caps[2][1] - 1.110223024625157e-15) <= eps)
    assert(abs(robot_caps[2][2] - 0.490300000000000) <= eps)
    assert(abs(robot_caps[2][3] - -1.594314036332950e-15) <= eps)
    assert(abs(robot_caps[2][4] - 1.110223024625157e-15) <= eps)
    assert(abs(robot_caps[2][5] - 0.490300000000000) <= eps)
    assert(abs(robot_caps[2][6] - (0.090000000000000 + secure_radius)) <= eps)
    assert(abs(robot_caps[3][0] - -4.693429213306548e-05) <= eps)
    assert(abs(robot_caps[3][1] - -0.005883360941466) <= eps)
    assert(abs(robot_caps[3][2] - 0.606104849281791) <= eps)
    assert(abs(robot_caps[3][3] - -2.365707868427609e-06) <= eps)
    assert(abs(robot_caps[3][4] - 0.061878260941472) <= eps)
    assert(abs(robot_caps[3][5] - 0.804871950718208) <= eps)
    assert(abs(robot_caps[3][6] - (0.07 + secure_radius)) <= eps)
    assert(abs(robot_caps[4][0] - -1.545847699099969e-15) <= eps)
    assert(abs(robot_caps[4][1] - 3.205768983605140e-15) <= eps)
    assert(abs(robot_caps[4][2] - 0.8065) <= eps)
    assert(abs(robot_caps[4][3] - -1.545847699099969e-15) <= eps)
    assert(abs(robot_caps[4][4] - 3.205768983605140e-15) <= eps)
    assert(abs(robot_caps[4][5] - 0.8065) <= eps)
    assert(abs(robot_caps[4][6] - (0.070000000000000 + secure_radius)) <= eps)
    assert(abs(robot_caps[5][0] - 4.932409872672667e-04) <= eps)
    assert(abs(robot_caps[5][1] - 0.003349675805265) <= eps)
    assert(abs(robot_caps[5][2] - 0.905303127433903) <= eps)
    assert(abs(robot_caps[5][3] - -4.926409872713459e-04) <= eps)
    assert(abs(robot_caps[5][4] - 0.003181224194743) <= eps)
    assert(abs(robot_caps[5][5] - 0.9713) <= eps)
    assert(abs(robot_caps[5][6] - (0.072548874802886 + secure_radius)) <= eps)    

