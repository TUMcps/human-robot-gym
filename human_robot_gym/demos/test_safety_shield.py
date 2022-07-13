"""This script shows an example of the Schunk robot being safely controlled in an human environment.

For instance, this can be used with our provided training function to train a safe RL agent.
"""

from safety_shield_py import Motion
from safety_shield_py import SafetyShield

from human_robot_gym.utils.mjcf_utils import file_path_completion

if __name__ == "__main__":
    trajectory_parameters_file = file_path_completion(
        "controllers/failsafe_controller/sara-shield/safety_shield/config/trajectory_parameters_schunk.yaml")
    robot_config_file = file_path_completion(
        "controllers/failsafe_controller/sara-shield/safety_shield/config/robot_parameters_schunk.yaml")
    mocap_config_file = file_path_completion(
        "controllers/failsafe_controller/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml")

    safety_shield = SafetyShield(
            activate_shield=True,
            sample_time=0.004,
            trajectory_config_file=trajectory_parameters_file,
            robot_config_file=robot_config_file,
            mocap_config_file=mocap_config_file,
            init_x=0.0,
            init_y=0.0,
            init_z=0.0,
            init_roll=0.0,
            init_pitch=0.0,
            init_yaw=0.0,
            init_qpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    human_measurement = [
        [10.0, 10.0, 0.0] for i in range(21)
    ]

    command_motion = Motion(0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t = 0
    for ep in range(1000):
        for step in range(100000):
            t += 0.001
            safety_shield.humanMeasurement(human_measurement, t)
            t += 0.003
            if step % 10000 == 0:
                p = [0.2 * t, 0.0, 0.0, 0.0, 0.0, 0.0]
                safety_shield.newLongTermTrajectory(p, v)
            desired_motion = safety_shield.step(t)
            desired_qpos = desired_motion.getAngle()
            desired_qvel = desired_motion.getVelocity()
            desided_qacc = desired_motion.getAcceleration()

        t = 0
        safety_shield.reset(activate_shield=True,
                            init_x=0.0,
                            init_y=0.0,
                            init_z=0.0,
                            init_roll=0.0,
                            init_pitch=0.0,
                            init_yaw=0.0,
                            init_qpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            current_time=t)
