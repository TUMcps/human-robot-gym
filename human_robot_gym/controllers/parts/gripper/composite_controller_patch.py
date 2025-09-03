"""
Monkey patch for robosuite CompositeController to work with waypoint gripper controllers.
"""

import numpy as np
from robosuite.controllers.composite.composite_controller import CompositeController


def patched_setup_action_split_idx(self):
    """Patched version of CompositeController.setup_action_split_idx fn to work with waypoint gripper controllers."""
    if hasattr(self.part_controllers[next(iter(self.part_controllers))], "use_waypoints_action") and \
       self.part_controllers[next(iter(self.part_controllers))].use_waypoints_action:
        # Waypoint implementation
        action_dim = self.action_limits[0].shape[0]
        n_waypoints = self.part_controllers[list(self.grippers.keys())[0]].n_waypoints
        assert action_dim % n_waypoints == 0, (
            f"Action dimension {action_dim} not divisible by number of waypoints {n_waypoints}"
        )
        base_action_dim = action_dim // n_waypoints
        previous_idx = 0
        last_idx = 0
        for part_name, controller in self.part_controllers.items():
            assert hasattr(controller, "n_waypoints"), (
                f"All controllers must be waypoint controllers (have property n_waypoints). {part_name} is not."
            )
            assert controller.n_waypoints == n_waypoints, (
                f"All waypoint controllers must have the same number of waypoints. {part_name} has \
                  {controller.n_waypoints}, expected {n_waypoints}."
            )
            if part_name in self.grippers.keys():
                last_idx += self.grippers[part_name].dof
            else:
                assert controller.control_dim % controller.n_waypoints == 0, (
                    f"Controller control dimension {controller.control_dim} not divisible by number of waypoints \
                      {controller.n_waypoints}"
                )
                last_idx += controller.control_dim // n_waypoints
            start_indices = []
            end_indices = []
            for i in range(n_waypoints):
                start_indices.append(previous_idx + i * base_action_dim)
                end_indices.append(last_idx + i * base_action_dim)
            self._action_split_indexes[part_name] = (start_indices, end_indices)
            previous_idx = last_idx
    else:
        # Regular implementation
        previous_idx = 0
        last_idx = 0
        for part_name, controller in self.part_controllers.items():
            if part_name in self.grippers.keys():
                last_idx += self.grippers[part_name].dof
            else:
                last_idx += controller.control_dim
            self._action_split_indexes[part_name] = (previous_idx, last_idx)
            previous_idx = last_idx


def patched_set_goal(self, all_action):
    """Patched version of CompositeController.set_goal function to work with waypoint gripper controllers."""
    if hasattr(self.part_controllers[next(iter(self.part_controllers))], "use_waypoints_action") and \
       self.part_controllers[next(iter(self.part_controllers))].use_waypoints_action:
        action_dim = self.action_limits[0].shape[0]
        n_waypoints = self.part_controllers[list(self.grippers.keys())[0]].n_waypoints
        assert action_dim % n_waypoints == 0, (
            f"Action dimension {action_dim} not divisible by number of waypoints {n_waypoints}"
        )
        # Waypoint implementation
        for part_name, controller in self.part_controllers.items():
            assert hasattr(controller, "n_waypoints"), (
                f"All controllers must be waypoint controllers (have property n_waypoints). {part_name} is not."
            )
            assert controller.n_waypoints == n_waypoints, (
                f"All waypoint controllers must have the same number of waypoints. {part_name} has \
                  {controller.n_waypoints}, expected {n_waypoints}."
            )
            start_idxs, end_idxs = self._action_split_indexes[part_name]
            assert controller.control_dim % controller.n_waypoints == 0, (
                f"Controller control dimension {controller.control_dim} not divisible by number of waypoints \
                  {controller.n_waypoints}"
            )
            action = np.zeros((controller.control_dim,))
            for i in range(controller.n_waypoints):
                part_waypoint_action = all_action[start_idxs[i]:end_idxs[i]]
                base_control_dim = controller.control_dim // controller.n_waypoints
                if part_name in self.grippers.keys():
                    part_waypoint_action = self.grippers[part_name].format_action(part_waypoint_action)
                action[i*base_control_dim:(i+1)*base_control_dim] = part_waypoint_action
            controller.set_goal(action)
    else:
        # Regular implementation
        for part_name, controller in self.part_controllers.items():
            start_idx, end_idx = self._action_split_indexes[part_name]
            action = all_action[start_idx:end_idx]
            if part_name in self.grippers.keys():
                action = self.grippers[part_name].format_action(action)
            controller.set_goal(action)


def patched_action_limits(self):
    """Patched version of CompositeController.action_limits property to work with waypoint gripper controllers."""
    low, high = [], []
    for part_name, controller in self.part_controllers.items():
        if part_name not in self.arms:
            if part_name in self.grippers.keys():
                if hasattr(controller, 'n_waypoints'):
                    # Waypoint gripper controller case
                    low_g, high_g = ([-1] * self.grippers[part_name].dof * controller.n_waypoints,
                                     [1] * self.grippers[part_name].dof * controller.n_waypoints)
                else:
                    # Regular gripper controller case
                    low_g, high_g = ([-1] * self.grippers[part_name].dof,
                                     [1] * self.grippers[part_name].dof)
                low, high = np.concatenate([low, low_g]), np.concatenate([high, high_g])
            else:
                control_dim = controller.control_dim
                low_c, high_c = ([-1] * control_dim, [1] * control_dim)
                low, high = np.concatenate([low, low_c]), np.concatenate([high, high_c])
        else:
            low_c, high_c = controller.control_limits
            low, high = np.concatenate([low, low_c]), np.concatenate([high, high_c])
    return low, high


def apply_composite_controller_patch():
    """Apply the monkey patch to CompositeController.action_limits"""
    CompositeController.action_limits = property(patched_action_limits)
    CompositeController.setup_action_split_idx = patched_setup_action_split_idx
    CompositeController.set_goal = patched_set_goal
