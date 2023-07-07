"""This file describes a collaborative lifting task for a single robot arm in a human environment.

The objective for the robot is to keep a board level while the human is lifting it from the opposite side.

If the tilt of the object exceeds a certain threshold, the episode is terminated.

Author
    Felix Trost (FT)

Changelog:
    07.07.23 FT File creation
"""
from typing import Any, Dict, List, Optional, OrderedDict, Tuple, Union

import xml.etree.ElementTree as ET

import numpy as np
from robosuite.utils.observables import Observable, sensor
from scipy.spatial.transform import Rotation

import mujoco_py

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.placement_samplers import ObjectPositionSampler
import robosuite.utils.transform_utils as T
from robosuite.utils.mjcf_utils import find_elements

from human_robot_gym.environments.manipulation.human_env import COLLISION_TYPE, HumanEnv
from human_robot_gym.utils.mjcf_utils import xml_path_completion, rot_to_quat, quat_to_rot


class CollaborativeLiftingCart(HumanEnv):
    """Collaborative lifting environment for a human-like and a robotic agent.

    The objective for the robot is to keep a board level while the human is lifting it from the opposite side.

    If the tilt of the object exceeds a certain threshold, the episode is terminated.

    Args:
        robots (str | List[str]): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: `"Sawyer"` would generate one arm; `["Panda", "Panda", "Sawyer"]` would generate three robot arms)
            Note: Must be a single single-arm robot!

        robot_base_offset (None | List[float] | List[List[float]]): Offset (x, y, z) of the robot bases.
            If more than one robot is loaded provide a list of doubles, one for each robot.
            Specify `None` for an offset of (0, 0, 0) for each robot.

        env_configuration (str): Specifies how to position the robots within the environment (default is `"default"`).
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (None | str | List[Dict[str, Any]]): If set, contains relevant controller parameters
            for creating a custom controller. Else, uses the default controller for this specific task.
            Should either be single dict if same controller is to be used for all robots or else it should be
            a list of the same length as `robots` param

        gripper_types (str | List[str]): type of gripper, used to instantiate
            gripper models from gripper factory. Default is `"default"`, which is the default grippers(s) associated
            with the robot(s) the `robots` specification. `None` removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single `str` if same gripper type is to be used for all
            robots or else it should be a list of the same length as the `robots` param

        initialization_noise (Dict[str, Any] | List[Dict[str, Any]]): Dict containing the initialization noise
            parameters. The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or `0.0` results in no noise being applied.
                If `"gaussian"` type of noise is applied then this magnitude scales the standard deviation applied,
                If `"uniform"` type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify `"gaussian"` or `"uniform"`

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as `robots` param

            :Note: Specifying `"default"` will automatically use the default noise settings.
                Specifying `None` will automatically create the required dict with `"magnitude"` set to `0.0`.

        table_full_size (Tuple[float, float, float]): x, y, and z dimensions of the table.

        table_friction (Tuple[float, float, float]): the three mujoco friction parameters for
            the table.

        board_full_size (Tuple[float, float, float]): x, y, and z dimensions of the board in which the nail sticks.

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (None | float): Scales the normalized reward function by the amount specified.
            If `None`, environment reward remains unnormalized

        collision_reward (float): Reward to be given in the case of a collision.

        task_reward (float): Reward to be given in the case of reaching the goal.

        min_balance (float): If the dot product between the board's normal and the up vector is smaller than this
            value, the episode is terminated.

        obstacle_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place obstacles on every reset, else a `UniformRandomSampler`
            is used by default.
            Obstacles are elements that should be avoided.

        has_renderer (bool): If `True`, render the simulation state in
            a viewer instead of headless mode.

        has_offscreen_renderer (bool): `True` if using off-screen rendering

        render_camera (str): Name of camera to render if `has_renderer` is `True`. Setting this value to `None`
            will resul` in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse

        render_collision_mesh (bool): `True` if rendering collision meshes in camera. `False` otherwise.

        render_visual_mesh (bool): `True` if rendering visual meshes in camera. `False` otherwise.

        render_gpu_device_id (int): corresponds to the GPU device id to use for offscreen rendering.
            Defaults to `-1`, in which case the device will be inferred from environment variables
            (`GPUS` or `CUDA_VISIBLE_DEVICES`).

        control_freq (float): how many control signals to receive in every second. This sets the amount of
            simulation time that passes between every action input.

        horizon (int): Every episode lasts for exactly `horizon` action steps.

        ignore_done (bool): `True` if never terminating the environment (ignore `horizon`).

        hard_reset (bool): If `True`, re-loads model, sim, and render object upon a `reset` call, else,
            only calls `self.sim.reset` and resets all robosuite-internal variables

        camera_names (str | List[str]): name of camera to be rendered. Should either be single `str` if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if `use_camera_obs` is `True`.

            :Note: To render all robots' cameras of a certain type (e.g.: `"robotview"` or `"eye_in_hand"`), use the
                convention `"all-{name}"` (e.g.: `"all-robotview"`) to automatically render all camera images from each
                robot's camera list).

        camera_heights (int | List[int]): height of camera frame. Should either be single `int` if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            `camera_names` param.

        camera_widths (int | List[int]): width of camera frame. Should either be single `int` if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            `camera_names` param.

        camera_depths (bool | List[bool]): `True` if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            `camera_names` param.

        camera_segmentations (None | str | List[str] | List[List[str]]): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not `None`, multiple types of segmentations can be specified. A [List[str] / str | None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A List[List[str]] specifies per-camera
            segmentation setting(s) to use.

        renderer (str): string for the renderer to use

        renderer_config (dict): dictionary for the renderer configurations

        use_failsafe_controller (bool): Whether or not the safety shield / failsafe controller should be active

        visualize_failsafe_controller (bool): Whether or not the reachable sets of the failsafe controller should be
            visualized

        visualize_pinocchio (bool): Whether or not pinocchio (collision prevention static env) should be visualized

        control_sample_time (float): Control frequency of the failsafe controller

        human_animation_names (List[str]): Human animations to play

        base_human_pos_offset (List[float]): Base human animation offset

        human_animation_freq (float): Speed of the human animation in fps.

        human_rand (List[float]): Max. randomization of the human [x-pos, y-pos, z-angle]

        n_animations_sampled_per_100_steps (int): How many animations to sample at resets per 100 steps in the horizon.
            After all animations of the list have been played, restart from the first animation in the list.
            This is done to ensure the same list of animations can be played when loading the env state from a file.

        safe_vel (float): Safe cartesian velocity. The robot is allowed to move with this velocity in the vicinity of
            humans.

        self_collision_safety (float): Safe distance for self collision detection

        seed (int): Random seed for `np.random`

        verbose (bool): If `True`, print out debug information

        done_at_collision (bool): If `True`, the episode is terminated when a collision occurs

        done_at_success (bool): If `True`, the episode is terminated when the goal is reached

    Raises:
        AssertionError: [Invalid number of robots specified]
    """
    def __init__(
        self,
        robots: Union[str, List[str]],
        robot_base_offset: Optional[Union[List[float], List[List[float]]]] = None,
        env_configuration: str = "default",
        controller_configs: Optional[Union[str, List[Dict[str, Any]]]] = None,
        gripper_types: Union[str, List[str]] = "default",
        initialization_noise: Union[str, List[str], List[Dict[str, Any]]] = "default",
        table_full_size: Tuple[float, float, float] = (0.4, 1.5, 0.05),
        table_friction: Tuple[float, float, float] = (1.0, 5e-3, 1e-4),
        board_full_size: Tuple[float, float, float] = (1.0, 0.5, 0.03),
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        collision_reward: float = -10,
        task_reward: float = 1,
        min_balance: float = 0.8,
        obstacle_placement_initializer: Optional[ObjectPositionSampler] = None,
        has_renderer: bool = False,
        has_offscreen_renderer: bool = True,
        render_camera: str = "frontview",
        render_collision_mesh: bool = False,
        render_visual_mesh: bool = True,
        render_gpu_device_id: int = -1,
        control_freq: float = 10,
        horizon: int = 1000,
        ignore_done: bool = False,
        hard_reset: bool = True,
        camera_names: Union[str, List[str]] = "frontview",
        camera_heights: Union[int, List[int]] = 256,
        camera_widths: Union[int, List[int]] = 256,
        camera_depths: Union[bool, List[bool]] = False,
        camera_segmentations: Optional[Union[str, List[str], List[List[str]]]] = None,
        renderer: str = "mujoco",
        renderer_config: Dict[str, Any] = None,
        use_failsafe_controller: bool = True,
        visualize_failsafe_controller: bool = False,
        visualize_pinocchio: bool = False,
        control_sample_time: float = 0.004,
        human_animation_names: List[str] = [
            "CollaborativeLifting/0",
            # "CollaborativeLifting/1",
            # "CollaborativeLifting/2",
            # "CollaborativeLifting/3",
            # "CollaborativeLifting/4",
            # "CollaborativeLifting/5",
            # "CollaborativeLifting/6",
            # "CollaborativeLifting/7",
            # "CollaborativeLifting/8",
            # "CollaborativeLifting/9",
            # "CollaborativeLifting/10",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 30,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        n_animations_sampled_per_100_steps: int = 5,
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
    ):
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.board_full_size = board_full_size

        self.table_offset = np.array([1.0, 0.0, 0.85])

        self.reward_scale = reward_scale
        self.collision_reward = collision_reward
        self.task_reward = task_reward
        self.min_balance = min_balance

        self.object_placement_initializer = None  # TODO
        self.obstacle_placement_initializer = obstacle_placement_initializer
        self.board = None
        self.board_body_id = None

        # The board should remain in the gripper at all times. If there are too many consecutive steps
        # where this is not the case, the episode is terminated.
        self._n_steps_without_gripped_board = 0

        self.done_at_collision = done_at_collision
        self.done_at_success = done_at_success

        self._animation_complete = False

        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
            use_failsafe_controller=use_failsafe_controller,
            visualize_failsafe_controller=visualize_failsafe_controller,
            visualize_pinocchio=visualize_pinocchio,
            control_sample_time=control_sample_time,
            human_animation_names=human_animation_names,
            base_human_pos_offset=base_human_pos_offset,
            human_animation_freq=human_animation_freq,
            human_rand=human_rand,
            n_animations_sampled_per_100_steps=n_animations_sampled_per_100_steps,
            safe_vel=safe_vel,
            self_collision_safety=self_collision_safety,
            seed=seed,
            verbose=verbose,
        )

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Step the simulation forward one timestep.

        The gripper action is disabled and replaced by a 'close gripper' action.
        The action space is not changed by this modification, the gripper actuation can be removed
        from the action space in a wrapper if desired.

        Args:
            action (np.ndarray): The action to take in the environment.

        Returns:
            Tuple[np.ndarray, float, bool, Dict[str, Any]]: The observation, reward, done flag, and info dict.
        """
        action[-1] = 1  # Always close the gripper
        obs, rew, done, info = super().step(action)

        if self.goal_reached:
            self._on_goal_reached()
            self.goal_reached = False

        if self.has_renderer:
            self._visualize()

        return obs, rew, done, info

    def _get_achieved_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        # TODO
        return np.concatenate(
            [
                [observation["board_balance"]],
                [observation["board_gripped"]],
            ]
        ).tolist()

    def _get_desired_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        robot_prefix = self.robots[0].robot_model.naming_prefix

        return np.concatenate(
            [
                [observation[f"{robot_prefix}eef_pos"]],
            ]
        ).tolist()

    def _check_success(
        self,
        achieved_goal: List[float],
        desired_goal: List[float]
    ) -> bool:
        return self._animation_complete

    def reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        reward = -1.0

        if self._check_success(achieved_goal=achieved_goal, desired_goal=desired_goal):
            reward = self.task_reward

        # TODO

        # Add a penalty for self-collisions and collisions with the human
        if COLLISION_TYPE(info["collision_type"]) not in (COLLISION_TYPE.NULL | COLLISION_TYPE.ALLOWED):
            reward += self.collision_reward

        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

    def _check_done(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> bool:
        balance = achieved_goal[0]
        board_gripped = achieved_goal[1]

        if board_gripped:
            self._n_steps_without_gripped_board = 0
        else:
            self._n_steps_without_gripped_board += 1

        if balance < self.min_balance:
            if self.verbose:
                print("Episode terminated due to board being unbalanced.")
            return True
        if self._n_steps_without_gripped_board > 5:
            if self.verbose:
                print("Episode terminated due to board not being gripped.")
            return True

        if self.done_at_collision and COLLISION_TYPE(info["collision_type"]) not in (
            COLLISION_TYPE.NULL | COLLISION_TYPE.ALLOWED
        ):
            return True

        success = self._check_success(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
        )

        if self.done_at_success and success:
            return True

        return False

    def _compute_animation_time(self, control_time: int) -> int:
        animation_time = super()._compute_animation_time(control_time)

        animation_length = self.human_animation_data[self.human_animation_id][0]["Pelvis_pos_x"].shape[0]

        if animation_time >= animation_length - 1:
            self._animation_complete = True
            animation_time = animation_length - 1

        return animation_time

    def _control_human(self, force_update: bool = True):
        super()._control_human(force_update=True)

        lh_rot = quat_to_rot(self.sim.data.get_body_xquat("Human_L_Hand"))
        rh_rot = quat_to_rot(self.sim.data.get_body_xquat("Human_R_Hand"))

        lh_quat = rot_to_quat(lh_rot * Rotation.from_euler("y", -np.pi / 2))
        rh_quat = rot_to_quat(rh_rot * Rotation.from_euler("y", np.pi / 2))

        self.sim.data.set_mocap_pos(
            "lh_mocap_object",
            self.sim.data.get_site_xpos("Human_L_Hand")
        )

        self.sim.data.set_mocap_pos(
            "rh_mocap_object",
            self.sim.data.get_site_xpos("Human_R_Hand")
        )

        self.sim.data.set_mocap_quat(
            "lh_mocap_object",
            lh_quat,
        )

        self.sim.data.set_mocap_quat(
            "rh_mocap_object",
            rh_quat,
        )

    def _change_grip_equalities_active(self, active: bool):
        self.sim.model.eq_active[self.eq_l_id] = active
        self.sim.model.eq_active[self.eq_r_id] = active

    def _human_pickup_object(self):
        self._change_grip_equalities_active(True)

    def _human_drop_object(self):
        self._change_grip_equalities_active(False)

    def _on_goal_reached(self):
        if not self.done_at_success:
            self.robots[0].reset(deterministic=True)
            self._reset_controller()
            self._reset_pin_models()
            self._progress_to_next_animation(
                animation_start_time=int(self.low_level_time / self.human_animation_step_length)
            )

    def _reset_animation(self):
        self._animation_complete = False
        self._n_steps_without_gripped_board = 0

        self.sim.data.set_joint_qpos(
            "board_joint0",
            np.concatenate(
                [
                    self.table_offset + np.array([0, 0, 0.05]) + self.human_pos_offset,
                    [0, 0, 0, 1],
                ]
            )
        )

    def _progress_to_next_animation(self, animation_start_time: float):
        super()._progress_to_next_animation(animation_start_time=animation_start_time)
        self._control_human()
        self._reset_animation()

    def _reset_internal(self):
        # Set the desired new initial joint angles before resetting the robot.
        self.robots[0].init_qpos = np.array([0, np.pi * 19 / 48, -np.pi / 2 - 5 * np.pi/48, 0, np.pi / 2, -np.pi / 4])

        super()._reset_internal()

        self._control_human()
        self._reset_animation()

    def reset(self) -> OrderedDict[str, Any]:
        obs = super().reset()

        # Try 2 steps to grasp the board, reset again if unsuccessful
        for i in range(2):
            obs, _, done, _ = self.step(np.concatenate(
                [
                    [0 for _ in range(self.action_dim - 1)],
                    [1],
                ]
            ))

            if done:
                if self.verbose:
                    print(f"Episode done after {i} steps. Animation id: {self.human_animation_id}")
                return self.reset()

            if self._check_grasp(
                gripper=self.robots[0].gripper,
                object_geoms=self.board,
            ):
                return obs

        if self.verbose:
            print(f"Episode terminated due to unsuccessful grasp. Animation id: {self.human_animation_id}")

        return self.reset()

    def _setup_arena(self):
        """Setup the mujoco arena.

        Must define `self.mujoco_arena`.
        Defines `self.objects` and `self.obstacles`.
        """
        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_offset=self.table_offset,
            xml=xml_path_completion("arenas/table_arena.xml"),
        )

        self._set_origin()

        self._set_mujoco_camera()

        board_size = np.array(self.board_full_size) * 0.5

        self.board = BoxObject(
            name="board",
            size=board_size,
            rgba=[0.6, 0.6, 0.0, 1],
            density=20,
        )

        self.objects = [
            self.board,
        ]

        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=[],
        )

        # << OBSTACLES >>
        self._setup_collision_objects(
            add_table=True,
            add_base=True,
            safety_margin=0.0
        )
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def _postprocess_model(self):
        super()._postprocess_model()

        r_anchor = "-0.45 -0.25, 0"
        l_anchor = "-0.45 0.25 0"

        lh_mocap_object = ET.Element(
            "body",
            name="lh_mocap_object",
            pos=l_anchor,
            quat="0 0 0 1",
            mocap="true",
        )

        lh_mocap_object.append(
            ET.Element(
                "geom",
                name="lh_mocap_object_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="0 1 1 0.5",
            )
        )

        rh_mocap_object = ET.Element(
            "body",
            name="rh_mocap_object",
            pos=r_anchor,
            quat="0 0 0 1",
            mocap="true",
        )

        rh_mocap_object.append(
            ET.Element(
                "geom",
                name="rh_mocap_object_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="1 0 1 0.5",
            )
        )

        self.model.worldbody.append(lh_mocap_object)
        self.model.worldbody.append(rh_mocap_object)

        box_elem = find_elements(
            root=self.model.root,
            tags="body",
            attribs={"name": "board_main"},
            return_first=True
        )

        l_grip = ET.Element(
            "body",
            name="lh_grip",
            pos=l_anchor,
        )

        r_grip = ET.Element(
            "body",
            name="rh_grip",
            pos=r_anchor,
        )

        l_grip.append(
            ET.Element(
                "geom",
                name="lh_grip_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="0 1 0 0.5",
            )
        )

        r_grip.append(
            ET.Element(
                "geom",
                name="rh_grip_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="1 0 0 0.5",
            )
        )

        box_elem.append(l_grip)
        box_elem.append(r_grip)

        self.model.equality.append(
            ET.Element(
                "connect",
                name="lh_mocap_object_connect",
                body1="lh_grip",
                body2="lh_mocap_object",
                anchor="0 0 0",
                active="true",
                solimp="-100 -100"
            )
        )

        self.model.equality.append(
            ET.Element(
                "connect",
                name="rh_mocap_object_connect",
                body1="rh_grip",
                body2="rh_mocap_object",
                anchor="0 0 0",
                active="true",
                solimp="-100 -100"
            )
        )

    def _setup_references(self):
        super()._setup_references()

        self.board_body_id = self.sim.model.body_name2id(self.board.root_body)
        self.eq_l_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, "lh_mocap_object_connect",
        )

        self.eq_r_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, "rh_mocap_object_connect",
        )

    def _setup_observables(self) -> OrderedDict[str, Observable]:
        observables = super()._setup_observables()

        robot_prefix = self.robots[0].robot_model.naming_prefix

        if (obs_key := f"{robot_prefix}joint_pos") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}joint_vel") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}eef_pos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := "human_joint_pos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}joint_pos_cos") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}joint_pos_sin") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}gripper_qpos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}gripper_qvel") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}eef_quat") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := "gripper_pos") in observables:
            observables[obs_key].set_active(False)

        goal_mod = "goal"
        obj_mod = "object"

        @sensor(modality=obj_mod)
        def board_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return np.array(self.sim.data.body_xpos[self.board_body_id])

        @sensor(modality=obj_mod)
        def board_quat(obs_cache: Dict[str, Any]) -> np.ndarray:
            return T.convert_quat(self.sim.data.body_xquat[self.board_body_id], to="xyzw")

        @sensor(modality=goal_mod)
        def board_balance(obs_cache: Dict[str, Any]) -> np.ndarray:
            if "board_quat" not in obs_cache:
                return np.zeros(1)
            else:
                balance = quat_to_rot(self.sim.data.body_xquat[self.board_body_id]).apply(
                    np.array([0, 0, 1])
                ).dot(np.array([0, 0, 1]))
                return balance

        @sensor(modality=goal_mod)
        def board_gripped(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self._check_grasp(
                gripper=self.robots[0].gripper,
                object_geoms=self.board,
            )

        @sensor(modality=obj_mod)
        def vec_eef_to_board(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["board_pos"] - np.array(obs_cache[f"{robot_prefix}eef_pos"]))
                if "board_pos" in obs_cache and f"{robot_prefix}eef_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=obj_mod)
        def quat_eef_to_board(obs_cache: Dict[str, Any]) -> np.ndarray:
            if "robot0_eef_quat" not in obs_cache or "object_quat" not in obs_cache:
                return np.zeros(4)

            quat = rot_to_quat(
                quat_to_rot(obs_cache["object_quat"]) * quat_to_rot(obs_cache["robot0_eef_quat"]).inv()
            )
            return T.convert_quat(np.array(quat), "xyzw")

        sensors = [
            board_pos,
            board_quat,
            board_balance,
            board_gripped,
            vec_eef_to_board,
            quat_eef_to_board,
        ]

        names = [s.__name__ for s in sensors]

        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )

        return observables

    def _visualize(self):
        """Visualize the goal space and the sampling space of initial object positions."""
        pass
