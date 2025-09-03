"""Extension of the simple gripper controller to waypoint actions."""

import numpy as np

from robosuite.controllers.parts.gripper.simple_grip import SimpleGripController


class SimpleWaypointGripController(SimpleGripController):
    """Extension of the simple gripper controller to waypoint actions.

    Args:
        n_waypoints: Number of waypoints in an action.
        n_control_steps_per_waypoint: For how many control steps should we use a gripper action.
            This is not ideal as the gripper action might diverge from the EEF pose.
            It would be better to check, where the robot currently is at its execution and then
            choose the correct gripper action for that position. However, that is more complex to
            implement and this implementation is already a bit hacky.
    """

    def __init__(
        self,
        joint_indexes,
        n_waypoints: int = 1,
        n_control_steps_per_waypoint: int = 1,
        **kwargs
    ):
        super().__init__(
          joint_indexes=joint_indexes,
          **kwargs
        )
        self.use_waypoints_action = True
        self.n_waypoints = n_waypoints
        self.base_control_dim = len(joint_indexes["actuators"])
        self.control_dim = self.base_control_dim * n_waypoints
        self.input_min = np.tile(self.input_min, n_waypoints)
        self.output_min = np.tile(self.output_min, n_waypoints)
        self.input_max = np.tile(self.input_max, n_waypoints)
        self.output_max = np.tile(self.output_max, n_waypoints)
        self.n_control_steps_per_waypoint = n_control_steps_per_waypoint
        self.control_step_counter = 0
        self.action_chunk = None

    def set_goal(self, action, set_qpos=None):
        """
        Sets goal based on input @action. If self.impedance_mode is not "fixed", then the input will be parsed into the
        delta values to update the goal position / pose and the kp and/or damping_ratio values to be immediately updated
        internally before executing the proceeding control loop.

        Note that @action expected to be in the following format, based on impedance mode!

            :Mode `'fixed'`: [joint pos command]
            :Mode `'variable'`: [damping_ratio values, kp values, joint pos command]
            :Mode `'variable_kp'`: [kp values, joint pos command]

        Args:
            action (Iterable): Desired relative joint position goal state
            set_qpos (Iterable): If set, overrides @action and sets the desired absolute joint position goal state

        Raises:
            AssertionError: [Invalid action dimension size]
        """
        # Update state
        self.update()

        # Parse action based on the impedance mode, and update kp / kd as necessary
        delta = action

        # Check to make sure delta is size self.joint_dim
        assert len(delta) == self.control_dim, (
            f"Delta qpos must be equal to the control dimension of the robot!"
            f"Expected {self.control_dim}, got {len(delta)}"
        )

        self.action_chunk = delta
        if self.use_action_scaling:
            self.action_chunk = self.scale_action(delta)

        self.update_goal(self.get_i_th_action(0))
        self.control_step_counter = 0

    def get_i_th_action(self, i: int = 0):
        """Return the i-th gripper action."""
        assert self.action_chunk is not None
        return self.action_chunk[i*self.base_control_dim:(i+1)*self.base_control_dim]

    def update_goal(self, sub_action):
        """Set the new sub goal."""
        self.goal_qvel = sub_action

        if self.interpolator is not None:
            self.interpolator.set_goal(self.goal_qvel)

    def run_controller(self):
        """
        Calculates the torques required to reach the desired setpoint

        Returns:
             np.array: Command torques
        """
        # Make sure goal has been set
        if self.goal_qvel is None:
            self.set_goal(np.zeros(self.control_dim))

        # Update state
        self.update()

        if self.control_step_counter % self.n_control_steps_per_waypoint == 0:
            i_act = int(self.control_step_counter/self.n_control_steps_per_waypoint)
            i_act = np.clip(i_act, 0, self.n_waypoints-1)
            self.update_goal(self.get_i_th_action(i_act))

        desired_qvel = None

        # Only linear interpolator is currently supported
        if self.interpolator is not None:
            # Linear case
            if self.interpolator.order == 1:
                desired_qvel = self.interpolator.get_interpolated_goal()
            else:
                # Nonlinear case not currently supported
                raise NotImplementedError("Nonlinear gripper controller not currently supported")
        else:
            desired_qvel = np.array(self.goal_qvel)

        self.vels = desired_qvel
        if self.use_action_scaling:
            ctrl_range = np.stack([self.actuator_min, self.actuator_max], axis=-1)
            bias = 0.5 * (ctrl_range[:, 1] + ctrl_range[:, 0])
            weight = 0.5 * (ctrl_range[:, 1] - ctrl_range[:, 0])
            self.vels = bias + weight * desired_qvel

        # Always run superclass call for any cleanups at the end
        super().run_controller()
        self.control_step_counter += 1
        if self.control_step_counter > self.n_waypoints * self.n_control_steps_per_waypoint:
            print(f"DEBUG: gripper command end reached with n steps = \
              {self.control_step_counter} > {self.n_waypoints * self.n_control_steps_per_waypoint}.")
        return self.vels

    def reset(self):
        """Reset the controller.

        TODO: Do we need this or does this change anything?
        """
        self.control_step_counter = 0
        self.action_chunk = None
        self.goal_qvel = None

    def reset_goal(self):
        """
        Resets joint position goal to be current position
        """
        self.goal_qvel = self.joint_vel

        # Reset interpolator if required
        if self.interpolator is not None:
            self.interpolator.set_goal(self.goal_qvel)

    @property
    def control_limits(self):
        """
        Returns the limits over this controller's action space, overrides the superclass property
        Returns the following (generalized for both high and low limits), based on the impedance mode:
        Returns:
            2-tuple:

                - (np.array) minimum action values
                - (np.array) maximum action values
        """
        return self.input_min, self.input_max

    @property
    def name(self):
        return "JOINT_VELOCITY_WAYPOINT"
