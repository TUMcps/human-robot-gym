"""This file implements an expert policy for the `PickPlaceHumanCart` environment.

The policy does not take the human into consideration,
but only the values defined in the `PickPlaceHumanCartExpertObservation` dataclass.

Author:
    Felix Trost (FT)

Changelog:
    08.02.23 FT File creation

"""
from typing import Any, Dict, Optional
from dataclasses import dataclass
import numpy as np

from gym.spaces import Box

from human_robot_gym.utils.ou_process import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts.expert import Expert


@dataclass
class PickPlaceHumanCartExpertObservation:
    """Data class to encapsulate all observation values relevant for the expert.

    Attributes:
        object_gripped (bool): whether both finger pads of the robot
            have contact with the object
        vec_eef_to_object (np.ndarray):
            vector from robot end effector to object
        vec_eef_to_target (np.ndarray):
            vector from robot end effector to target
        robot0_gripper_qpos (np.ndarray):
            joint positions of the two fingers
    """
    object_gripped: bool
    vec_eef_to_object: np.ndarray
    vec_eef_to_target: np.ndarray
    robot0_gripper_qpos: np.ndarray


class PickPlaceHumanCartExpert(Expert):
    """Expert policy for the `PickPlaceHumanCart` environment.

    Does not take the human into account.
    The relevant observation data is encapsulated in the `PickPlaceExpertObservation` data class.
    Behavior:

        - Move above object and open gripper
        - Move to object
        - Close gripper
        - Move above target
        - Move to target
        - If `release_when_delivered` is True:
            - Open gripper and move above object
        - Else:
            - Remain at target with gripper closed around object

    Noise can be added to the motion parameters. We draw from an Ornstein-Uhlenbeck (OU) process
    with asymptotic mean 0 and variance of half the motion action limit as described in the action space.
    Note that the OU process maintains a custom random number generator that is not affected by np.random.seed calls.
    Formula to obtain motion action parameters:
    `motion = expert_policy * signal_to_noise_ratio + noise * (1 - signal_to_noise_ratio)`

    Args:
        observation_space (Space): the environment observation space
        action_space (Space): the environment action space
        signal_to_noise_ratio (float): interpolation between
            noise signal (Ornstein-Uhlenbeck process) -> signal_to_noise_ratio = 0
            and expert policy -> signal_to_noise_ratio = 1
        hover_dist (float): vertical distance the expert should reach
            while moving above the next objective
        tan_theta (float): tangent of the opening angle of the cone describing
            points above the next objective the expert should reach
        horizontal_epsilon (float): maximum horizontal tolerance for the expert to be able to grip the object.
            May depend on the gripper type
        vertical_epsilon (float): maximum vertical tolerance for the expert to be able to grip the object;
            has to be chosen high enough to respect the safety margin to the table
            and low enough to ensure the object can be gripped. Depends on the gripper type
        gripper_fully_opened_threshold (float): minimum difference of both gripper joint positions
            at which the gripper is considered to be fully opened. Depends on the gripper type
        release_when_delivered (bool): whether the expert should release the object when it is delivered to the target
            This option is `True` per default. Setting it to `False` makes sense for example if the target is in the air
        delta_time (float): time between two calls of the expert policy. Only used to step the OU process
        seed (int): random seed for the noise signal
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        signal_to_noise_ratio: float = 1,
        hover_dist: float = 0.2,
        tan_theta: float = 0.5,
        horizontal_epsilon: float = 0.03,
        vertical_epsilon: float = 0.015,
        goal_dist: float = 0.08,
        gripper_fully_opened_threshold: float = 0.02,
        release_when_delivered: bool = True,
        delta_time: float = 0.01,
        seed: Optional[int] = None,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space,
        )

        self._motion_action_limit = action_space.high[0]
        self._gripper_action_limit = action_space.high[-1]

        # TODO fix init_qpos values and use gripper dependent values
        # self._gripper_fully_opened_threshold = gripper_model.init_qpos[0] - gripper_model.init_qpos[1]
        self._gripper_fully_opened_threshold = gripper_fully_opened_threshold
        self._horizontal_epsilon = horizontal_epsilon
        self._vertical_epsilon = vertical_epsilon
        self._goal_dist = goal_dist
        self._tan_theta = tan_theta
        self._hover_dist = hover_dist
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._release_when_delivered = release_when_delivered
        self._delta_time = delta_time
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0,
            sigma=self._motion_action_limit * 0.5,
            seed=seed,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select actions based on observations.

        Args:
            obs_dict (Dict): observation dictionary

        Returns:
            np.ndarray: action
        """
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)
        action = np.zeros(4)
        motion = self._select_motion(obs).clip(
            -self._motion_action_limit,
            self._motion_action_limit
        )

        # Interpolate between expert policy and noise signal
        action[:3] = (
            motion * self._signal_to_noise_ratio +
            self._motion_noise.step(dt=self._delta_time) * (1 - self._signal_to_noise_ratio)
        ).clip(-self._motion_action_limit, self._motion_action_limit)

        action[3] = self._select_gripper_action(obs).clip(
            -self._gripper_action_limit,
            self._gripper_action_limit,
        )

        return action

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> PickPlaceHumanCartExpertObservation:
        """Convert observation dictionary to `PickPlaceHumanCartExpertObservation` data object."""
        return PickPlaceHumanCartExpertObservation(
            object_gripped=obs_dict["object_gripped"],
            vec_eef_to_object=obs_dict["vec_eef_to_object"],
            vec_eef_to_target=obs_dict["vec_eef_to_target"],
            robot0_gripper_qpos=obs_dict["robot0_gripper_qpos"],
        )

    def _select_motion(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Select motion arguments of action (action[0:3]).

        To ensure the next objective can be reached (grip object or deliver object to target),
        the expert first moves towards a point a given distance above the next objective.
        If the gripper is sufficiently close to this point and some conditions for meeting the next objective are met
        (gripper fully opened if next objective is to grip the object,
        or object gripped if the next objective is to deliver it to the target),
        proceed down towards the next objective.
        If the object is within vicinity of the target and the gripper opened again, move the gripper above the object.
        """
        if self._object_delivered(obs) and self._gripper_fully_opened(obs):
            return self._move_to_above_object(obs=obs)
        elif self._above_object(obs) and self._gripper_fully_opened(obs):
            return self._move_to_object(obs)
        elif self._above_target(obs) and obs.object_gripped:
            return self._move_to_target(obs)
        elif obs.object_gripped:
            return self._move_to_above_target(obs)
        else:
            return self._move_to_above_object(obs)

    def _select_gripper_action(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Select gripper actuation argument of action (action[3]).

        Select the action to close the gripper if the object is gripped or can be gripped
        but was not yet delivered to the target.
        Otherwise (object not yet gripped, dropped or successfully delivered), select the action to open the gripper.

        If `self._release_when_delivered` is `False`,
        the gripper not is opened when the object is delivered to the target.
        """
        if self._object_delivered(obs) and self._release_when_delivered:
            return self._open_gripper()
        elif obs.object_gripped or self._at_object(obs):
            return self._close_gripper()
        else:
            return self._open_gripper()

    def _gripper_fully_opened(self, obs: PickPlaceHumanCartExpertObservation) -> bool:
        """Determine whether the gripper is opened further than a given threshold"""
        gripper_aperture = obs.robot0_gripper_qpos[0] - obs.robot0_gripper_qpos[1]
        return gripper_aperture > self._gripper_fully_opened_threshold

    def _at_object(self, obs: PickPlaceHumanCartExpertObservation) -> bool:
        """Determine whether the distance from gripper to the object is below a given threshold"""
        return (
            np.linalg.norm(obs.vec_eef_to_object[:2]) < self._horizontal_epsilon and
            -obs.vec_eef_to_object[2] < self._vertical_epsilon
        )

    def _object_delivered(self, obs: PickPlaceHumanCartExpertObservation) -> bool:
        """Determine whether the object is at the target location"""
        object_to_target = obs.vec_eef_to_target - obs.vec_eef_to_object
        return (
            np.linalg.norm(object_to_target[:2]) < self._horizontal_epsilon and
            np.abs(object_to_target[2]) < self._vertical_epsilon
        )

    def _is_within_truncated_cone(self, vec: np.ndarray) -> bool:
        """Determine if the vector points to a point within a truncated cone.

        This cone is characterized by its:
            - Minimum radius (self._horizontal_epsilon)
            - Opening angle (arctan(self._tan_theta))

        and is opened towards the negative z-axis.

        This function is used to define a volume in which the gripper is considered to be
        above the next objective (object and target locations).
        Opposed to a simple cylinder, this truncated cone gives more tolerance to the gripper at increasing
        vertical distances to the next objective.

        Args:
            vec (np.ndarray): vector pointing to the point to be checked
        Returns:
            bool: whether the point is within the truncated cone
        """
        max_radius = self._horizontal_epsilon - vec[2] * self._tan_theta
        return np.linalg.norm(vec[:2]) < max_radius and vec[2] < 0

    def _above_object(self, obs: PickPlaceHumanCartExpertObservation) -> bool:
        """Determine whether the gripper is within a truncated cone above the object.

        Minimum radius of the cone: self._horizontal_epsilon
        Cone angle: arctan(self._tan_theta)
        """
        return self._is_within_truncated_cone(vec=obs.vec_eef_to_object)

    def _above_target(self, obs: PickPlaceHumanCartExpertObservation) -> bool:
        """Determine whether the object is within a truncated cone above the target.

        Minimum radius of the cone: self._horizontal_epsilon
        Cone angle: arctan(self._tan_theta)
        """
        return self._is_within_truncated_cone(vec=obs.vec_eef_to_target)

    def _move_to_object(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Get the motion vector toward the object position"""
        return obs.vec_eef_to_object

    def _move_to_target(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Get the motion vector toward the target position"""
        return obs.vec_eef_to_target

    def _move_to_above_object(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Get the motion vector toward a point a given distance above the object"""
        return self._move_to_object(obs) + np.array([0, 0, self._hover_dist])

    def _move_to_above_target(self, obs: PickPlaceHumanCartExpertObservation) -> np.ndarray:
        """Get the motion vector toward a point a given distance above the target"""
        return self._move_to_target(obs) + np.array([0, 0, self._hover_dist])

    def _open_gripper(self) -> np.ndarray:
        """Get a gripper actuation for opening the gripper"""
        return np.ones(1) * -1

    def _close_gripper(self) -> np.ndarray:
        """Get a gripper actuation for closing the gripper"""
        return np.ones(1)
