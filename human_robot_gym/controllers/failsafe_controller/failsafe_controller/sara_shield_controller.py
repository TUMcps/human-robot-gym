"""SARA shield-based safety controller for human-robot interaction.

This controller implements the SARA (Safe Autonomous Robotic Assistant) shield
for ensuring safe robot operation in the vicinity of humans using reachability analysis.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2025-09-05 JT Refactored from original FailsafeController
"""

import numpy as np
from scipy.spatial.transform import Rotation

from human_robot_gym.controllers.failsafe_controller.failsafe_controller.safety_controller_base import SafetyController
from safety_shield_py import SafetyShield, ShieldType, ContactType, AABB
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.plot_capsule import PlotCapsule


class SaraShieldController(SafetyController):
    """SARA shield-based safety controller for human-robot interaction.

    This controller uses the SARA (Safe Autonomous Robotic Assistant) shield
    to ensure safety through reachability analysis and forward reachable sets.

    Additional Args (beyond SafetyController):
        All standard SafetyController arguments plus SARA-shield specific parameters
        are passed through kwargs and handled by the SafetyShield library.
    """

    def __init__(self, **kwargs):
        """Initialize SARA shield controller."""
        # Initialize parent class
        super().__init__(**kwargs)

        # SARA shield specific initialization
        import os
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # Setup workspace boundaries (table)
        self.table = AABB([self.base_pos[0]-0.75, self.base_pos[1]-1.0, 0.82 - 0.05],
                          [self.base_pos[0]+0.75, self.base_pos[1]+1.0, 0.82])

        # Setup shield and contact types
        self.shield_type_enum = eval("ShieldType." + self.shield_type)
        self.eef_contact_type = eval("ContactType." + "WEDGE")

        # Initialize SARA safety shield
        self.safety_shield = SafetyShield(
            sample_time=self.control_sample_time,
            trajectory_config_file=(
                f"{dir_path}/../sara-shield/safety_shield/config/trajectory_parameters_{self.robot_name.lower()}.yaml"
            ),
            robot_config_file=f"{dir_path}/../sara-shield/safety_shield/config/robot_parameters_{self.robot_name.lower()}.yaml",
            mocap_config_file=dir_path + f"/../sara-shield/safety_shield/config/{self.mocap_file}",
            init_x=self.base_pos[0],
            init_y=self.base_pos[1],
            init_z=self.base_pos[2],
            init_roll=self.base_orientation[0],
            init_pitch=self.base_orientation[1],
            init_yaw=self.base_orientation[2],
            init_qpos=self.init_qpos,
            environment_elements=[self.table],
            shield_type=self.shield_type_enum,
            eef_contact_type=self.eef_contact_type
        )

        # Initialize desired motion
        self.desired_motion = self.safety_shield.step(0.0)

        # Visualization capsules
        self.robot_capsules = []
        self.human_capsules = []
        self.robot_cap_in = []
        self.human_cap_in = []

        # Command velocity placeholder
        self.command_vel = [0.0 for i in self.init_qpos]

    def reset(self, base_pos=[0.0, 0.0, 0.0], base_orientation=[0.0, 0.0, 0.0, 1.0], shield_type="SSM"):
        """Reset the SARA shield controller.

        Args:
            base_pos (list): Robot base position [x, y, z]
            base_orientation (list): Robot base orientation [x, y, z, w]
            shield_type (str): Shield type ("SSM", "PFL", "OFF")
        """
        # Reset parent
        self.goal_qpos = None
        self.torques = None
        self.new_update = True
        self.sim.forward()
        self.update()
        self.joint_pos = np.array(self.sim.data.qpos[self.qpos_index])
        self.initial_joint = self.joint_pos
        self.initial_ee_pos = self.ee_pos
        self.initial_ee_ori_mat = self.ee_ori_mat
        
        # Update base transform
        rot = Rotation.from_quat([
            base_orientation[0], base_orientation[1],
            base_orientation[2], base_orientation[3]
        ])
        self.base_pos = np.array(base_pos)
        self.base_orientation = rot.as_euler("XYZ")
        
        # Update shield type
        self.shield_type = shield_type
        self.shield_type_enum = eval("ShieldType." + shield_type)
        
        # Reset SARA safety shield
        self.safety_shield.reset(
            init_x=self.base_pos[0],
            init_y=self.base_pos[1],
            init_z=self.base_pos[2],
            init_roll=self.base_orientation[0],
            init_pitch=self.base_orientation[1],
            init_yaw=self.base_orientation[2],
            init_qpos=self.joint_pos,
            current_time=self.sim.data.time,
            environment_elements=[self.table],
            shield_type=self.shield_type_enum,
            eef_contact_type=self.eef_contact_type
        )

        # Reset safety state
        self.safety_intervention = False

    def set_human_measurement(self, human_measurement, time):
        """Set the human measurement for the SARA safety shield.

        Args:
            human_measurement (list): List of human measurements [x, y, z]-joint positions.
                The order of joints is defined in the motion capture config file.
            time (float): Time of the human measurement
        """
        self.safety_shield.humanMeasurement(human_measurement, time)

    def get_safety(self):
        """Return if the SARA shield intervened in this step.

        Returns:
            bool: True if safe, False if unsafe
        """
        return self.safety_shield.getSafety()

    def get_robot_capsules(self):
        """Return the robot capsules for visualization.

        Returns:
            list: List of robot capsules in mujoco format
        """
        if self.shield_type_enum == ShieldType.OFF:
            return []
        self.robot_cap_in = self.safety_shield.getRobotReachCapsules()
        if len(self.robot_capsules) == 0:
            for cap in self.robot_cap_in:
                self.robot_capsules.append(PlotCapsule(cap[0:3], cap[3:6], cap[6]))
        else:
            assert len(self.robot_capsules) == len(self.robot_cap_in)
            for i in range(len(self.robot_cap_in)):
                self.robot_capsules[i].update_pos(
                    self.robot_cap_in[i][0:3], self.robot_cap_in[i][3:6], self.robot_cap_in[i][6]
                )
        return self.robot_capsules

    def get_human_capsules(self):
        """Return the human capsules for visualization.

        Returns:
            list: List of human capsules in mujoco format
        """
        if self.shield_type_enum == ShieldType.OFF:
            return []
        self.human_cap_in = self.safety_shield.getHumanReachCapsules(0)
        if len(self.human_capsules) == 0:
            for cap in self.human_cap_in:
                self.human_capsules.append(PlotCapsule(cap[0:3], cap[3:6], cap[6]))
        else:
            assert len(self.human_capsules) == len(self.human_cap_in)
            for i in range(len(self.human_cap_in)):
                self.human_capsules[i].update_pos(
                    self.human_cap_in[i][0:3], self.human_cap_in[i][3:6], self.human_cap_in[i][6]
                )
        return self.human_capsules

    # Abstract method implementations

    def _apply_safety_filter(self, desired_qpos):
        """Apply SARA shield safety filtering.
        
        Args:
            desired_qpos (np.ndarray): Desired joint positions
            
        Returns:
            np.ndarray: Safe joint positions
        """
        # Set the long-term trajectory goal
        self.safety_shield.newLongTermTrajectory(desired_qpos, self.command_vel)
        return desired_qpos  # SARA shield handles safety internally

    def _handle_waypoint_action(self, action):
        """Handle waypoint-based actions with SARA shield.
        
        Args:
            action (np.ndarray): Waypoint action
        """
        # Convert to 2D list of waypoints
        action_2D = np.reshape(action, (self.n_waypoints, int(len(action)/self.n_waypoints)))
        if int(len(action)/self.n_waypoints) < 7:
            # Add a zero to all actions to have 7 dimensions for ruckig trajectory planning
            action_2D = np.hstack((action_2D, np.zeros((self.n_waypoints, 1))))
        elif int(len(action)/self.n_waypoints) > 7:
            raise NotImplementedError(
                f"Change DOF in planning_utils_ruckig_pro.cc to {int(len(action)/self.n_waypoints)}")
        
        self.goal_qpos = action_2D[0]  # First waypoint is the immediate goal
        self.safety_shield.newLongTermTrajectoryFromWaypoints(action_2D)

    def _get_desired_motion(self):
        """Get the desired motion from SARA shield.
        
        Returns:
            tuple: (desired_qpos, desired_qvel, desired_qacc)
        """
        current_time = self.sim.data.time
        self.desired_motion = self.safety_shield.step(current_time)
        
        # Get capsule information for visualization
        self.get_human_capsules()
        
        desired_qpos = self.desired_motion.getAngle()
        desired_qvel = self.desired_motion.getVelocity()
        desired_qacc = self.desired_motion.getAcceleration()
        
        return desired_qpos, desired_qvel, desired_qacc

    @property
    def name(self):
        """Return controller name."""
        return "SARA_SHIELD"