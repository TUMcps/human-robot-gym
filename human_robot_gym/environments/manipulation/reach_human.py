from collections import OrderedDict
from ntpath import join
from typing import Dict, Union, List

import numpy as np
import pickle
import math
import json

from mujoco_py import load_model_from_path

from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import TableArena
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler

from human_robot_gym.models.objects.human.human import HumanObject
from human_robot_gym.utils.mjcf_utils import xml_path_completion, rot_to_quat

from scipy.spatial.transform import Rotation

from human_robot_gym.controllers.failsafe_controller.failsafe_controller.failsafe_controller import FailsafeController

class ReachHuman(SingleArmEnv):
    """
    This class corresponds to the reaching task for a single robot arm in a human environment.

    Args:
        robots (str or list of str): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: "Sawyer" would generate one arm; ["Panda", "Panda", "Sawyer"] would generate three robot arms)
            Note: Must be a single single-arm robot!

        env_configuration (str): Specifies how to position the robots within the environment (default is "default").
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (str or list of dict): If set, contains relevant controller parameters for creating a
            custom controller. Else, uses the default controller for this specific task. Should either be single
            dict if same controller is to be used for all robots or else it should be a list of the same length as
            "robots" param

        gripper_types (str or list of str): type of gripper, used to instantiate
            gripper models from gripper factory. Default is "default", which is the default grippers(s) associated
            with the robot(s) the 'robots' specification. None removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single str if same gripper type is to be used for all
            robots or else it should be a list of the same length as "robots" param

        initialization_noise (dict or list of dict): Dict containing the initialization noise parameters.
            The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or 0.0 results in no noise being applied.
                If "gaussian" type of noise is applied then this magnitude scales the standard deviation applied,
                If "uniform" type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify "gaussian" or "uniform"

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as "robots" param

            :Note: Specifying "default" will automatically use the default noise settings.
                Specifying None will automatically create the required dict with "magnitude" set to 0.0.

        table_full_size (3-tuple): x, y, and z dimensions of the table.

        table_friction (3-tuple): the three mujoco friction parameters for
            the table.

        use_camera_obs (bool): if True, every observation includes rendered image(s)

        use_object_obs (bool): if True, include object information in the observation.

        reward_scale (None or float): Scales the normalized reward function by the amount specified.
            If None, environment reward remains unnormalized

        reward_shaping (bool): if True, use dense rewards, else use sparse rewards.

        placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a UniformRandomSampler
            is used by default.

        has_renderer (bool): If true, render the simulation state in
            a viewer instead of headless mode.

        has_offscreen_renderer (bool): True if using off-screen rendering

        render_camera (str): Name of camera to render if `has_renderer` is True. Setting this value to 'None'
            will result in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse

        render_collision_mesh (bool): True if rendering collision meshes in camera. False otherwise.

        render_visual_mesh (bool): True if rendering visual meshes in camera. False otherwise.

        render_gpu_device_id (int): corresponds to the GPU device id to use for offscreen rendering.
            Defaults to -1, in which case the device will be inferred from environment variables
            (GPUS or CUDA_VISIBLE_DEVICES).

        control_freq (float): how many control signals to receive in every second. This sets the amount of
            simulation time that passes between every action input.

        horizon (int): Every episode lasts for exactly @horizon action steps.

        ignore_done (bool): True if never terminating the environment (ignore @horizon).

        hard_reset (bool): If True, re-loads model, sim, and render object upon a reset call, else,
            only calls self.sim.reset and resets all robosuite-internal variables

        camera_names (str or list of str): name of camera to be rendered. Should either be single str if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if @use_camera_obs is True.

            :Note: To render all robots' cameras of a certain type (e.g.: "robotview" or "eye_in_hand"), use the
                convention "all-{name}" (e.g.: "all-robotview") to automatically render all camera images from each
                robot's camera list).

        camera_heights (int or list of int): height of camera frame. Should either be single int if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_widths (int or list of int): width of camera frame. Should either be single int if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_depths (bool or list of bool): True if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            "camera names" param.

        camera_segmentations (None or str or list of str or list of list of str): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not None, multiple types of segmentations can be specified. A [list of str / str or None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A list of list of str specifies per-camera
            segmentation setting(s) to use.

        use_failsafe_controller (bool): Whether or not the safety shield / failsafe controller should be active

        visualize_failsafe_controller (bool): Whether or not the reachable sets of the failsafe controller should be visualized

        control_sample_time (double): Control frequency of the failsafe controller

        human_animation_names (list[str]): Human animations to play

        base_human_pos_offset (list[double]): Base human animation offset

        human_animation_freq (double): Speed of the human animation in fps.

    Raises:
        AssertionError: [Invalid number of robots specified]
    """

    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise="default",
        table_full_size=(0.4, 0.8, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=10,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,  # {None, instance, class, element}
        renderer="mujoco",
        renderer_config=None,
        use_failsafe_controller=True,
        visualize_failsafe_controller=False,
        control_sample_time=0.004,
        human_animation_names=["62_01", "62_03", "62_03", "62_07", "62_09", "62_10", "62_12", "62_13", "62_14", "62_15", "62_16", "62_18", "62_19", "62_20", "62_21"],
        base_human_pos_offset=[0.0, 0.0, 0.0],
        human_animation_freq=120,
    ):
        self.failsafe_controller = None
        self.control_sample_time = control_sample_time
        self.use_failsafe_controller = use_failsafe_controller
        self.visualize_failsafe_controller = visualize_failsafe_controller
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        # settings for table top (hardcoded since it's not an essential part of the environment)
        self.table_offset = np.array((0.0, 0.0, 0.8))

        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        self.obj_names = ["Human"]

        # Human animation definition
        self.human_animation_names = human_animation_names
        self.animation_info = {}
        with open(xml_path_completion('human/animations/animation_info.json')) as json_file:
            self.animation_info = json.load(json_file)
        self.human_animations = []
        for animation_name in self.human_animation_names:
            try:
                pkl_file = open(xml_path_completion('human/animations/{}.pkl'.format(animation_name)), 'rb')
                self.human_animations.append(pickle.load(pkl_file))
                pkl_file.close()
                if animation_name not in self.animation_info:
                    self.animation_info[animation_name] = {
                        "position_offset": [0.0, 0.0, 0.0],
                        "orientation_quat": [0.0, 0.0, 0.0, 1.0]}
            except Exception as e:
                print("Error while loading human animation {}: {}".format(pkl_file, e))
        
        self.base_human_pos_offset = base_human_pos_offset
        # Input to scipy: quat = [x, y, z, w]
        self.human_base_quat = Rotation.from_quat([ 0.7071068, 0, 0, 0.7071068  ])
        self.human_animation_freq = human_animation_freq
        self.low_level_time = int(0)
        self.human_animation_id = 0
        self.animation_start_time = 0

        # object placement initializer
        self.placement_initializer = placement_initializer

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            mount_types="default",
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
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
        )

        # Override robot controller
        self._create_new_controller()
        self._override_controller()
        # Setup collision variables
        self._setup_collision_info()

    def step(self, action):
        """
        Overrides base.py step function to create an GoalEnv.
        Takes a step in simulation with control command @action.
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            4-tuple:
                - (OrderedDict) observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        Raises:
            ValueError: [Steps past episode termination]
        """
        if self.done:
            raise ValueError("executing action in terminated episode")

        self.timestep += 1

        # Since the env.step frequency is slower than the mjsim timestep frequency, the internal controller will output
        # multiple torque commands in between new high level action commands. Therefore, we need to denote via
        # 'policy_step' whether the current step we're taking is simply an internal update of the controller,
        # or an actual policy update
        policy_step = True

        # Loop through the simulation at the model timestep rate until we're ready to take the next policy step
        # (as defined by the control frequency specified at the environment level)
        for i in range(int(self.control_timestep / self.control_sample_time)):
            self.sim.forward()
            self._human_measurement()
            self._set_human_measurement(self.human_measurement, self.sim.data.time)
            # The first step i=0 is a policy step, the rest not.
            # Only in a policy step, set_goal of controller will be called.
            self._pre_action(action, policy_step)
            #print(self.robots[0].controller.goal_qpos)
            #print(self.robots[0].controller.joint_pos)
            # Step the simulation n times
            for n in range(int(self.control_sample_time/self.model_timestep)):
              self.sim.step()
              self._control_human()
              self.sim.forward()
            self._update_observables()
            collision = self._collision_detection()
            policy_step = False
            self.low_level_time += 1

        if self.visualize_failsafe_controller:
            self._visualize_reachable_sets()
        # Note: this is done all at once to avoid floating point inaccuracies
        self.cur_time += self.control_timestep

        if self.viewer_get_obs:
            #observations = self.viewer._get_observations()
            raise NotImplementedError
        else:
            observations = self._get_observations()

        info = self._get_info()
        achieved_goal = self._get_achieved_goal_from_obs(observations)
        desired_goal = self._get_desired_goal_from_obs(observations)
        reward = self._compute_reward(desired_goal, achieved_goal, info)
        self.done = self._compute_done(desired_goal, achieved_goal, info)

        if self.viewer is not None and self.renderer != "mujoco":
            self.viewer.update()

        return observations, reward, self.done, info

    def _get_info(self) -> Dict:
        """
        Return the info dictionary of this step.

        Returns
            - info dict containing of
                * 
        """
        return {}

    def _compute_reward(self,
        achieved_goal: Union[List[float], List[List[float]]], 
        desired_goal: Union[List[float], List[List[float]]], 
        info: Union[Dict, List[Dict]]) -> Union[float, List[float]]: 
        """
        Compute the reward based on the achieved goal, the desired goal, and
        the info dict.

        This function can either be called for one sample or a list of samples.
        Args:
            - achieved_goal: observation of robot state that is relevant for goal
            - desired_goal: the desired goal
            - info: dictionary containing additional information like collision
        Returns:
            - reward (list of rewards)
        """
        return 0

    def _compute_done(self,
        achieved_goal: Union[List[float], List[List[float]]], 
        desired_goal: Union[List[float], List[List[float]]], 
        info: Union[Dict, List[Dict]]) -> Union[bool, List[bool]]: 
        """
        Compute the done flag based on the achieved goal, the desired goal, and
        the info dict.

        This function can either be called for one sample or a list of samples.
        Args:
            - achieved_goal: observation of robot state that is relevant for goal
            - desired_goal: the desired goal
            - info: dictionary containing additional information like collision
        Returns:
            - done (list of dones)
        """
        done = (self.timestep >= self.horizon) and not self.ignore_done
        return done

    def _get_achieved_goal_from_obs(self,
        observation: Union[List[float], Dict]
        ) -> List[float]:
        """
        Extract the achieved goal from the observation.

        Args:
            - observation: The observation after the action is executed
        
        Returns:
            - The achieved goal
        """
        return [0]

    def _get_desired_goal_from_obs(self,
        observation: Union[List[float], Dict]
        ) -> List[float]:
        """
        Extract the desired goal from the observation.

        Args:
            - observation: The observation after the action is executed
        
        Returns:
            - The desired goal
        """
        return [0]

    def _setup_collision_info(self):
        """
        Setup variables for collision detection.
        """
        # Robot links
        self.robot_collision_geoms = {self.sim.model.geom_name2id(item) for item in self.robots[0].robot_model.contact_geoms}
        # Gripper elements
        for el in self.robots[0].gripper.contact_geoms:
            self.robot_collision_geoms.add(self.sim.model.geom_name2id(el))
        # Human elements
        self.human_collision_geoms = {self.sim.model.geom_name2id(item) for item in self.human.contact_geoms}
        

    def _collision_detection(self):
        """
        Detects collisions between robot and environment.

        Returns:
            - 0 for no collision
            - 1 for non-critical collision
            - 2 for critical collision
        """
        for i in range(self.sim.data.ncon):
            # Note that the contact array has more than `ncon` entries,
            # so be careful to only read the valid entries.
            contact = self.sim.data.contact[i]
            if ((contact.geom1 in self.robot_collision_geoms) or 
               (contact.geom2 in self.robot_collision_geoms)):
                if ((contact.geom1 in self.robot_collision_geoms) and 
                    (contact.geom2 in self.robot_collision_geoms)):
                    print('Self-collision detected between ', self.sim.model.geom_id2name(contact.geom1), ' and ', self.sim.model.geom_id2name(contact.geom2))
                elif ((contact.geom1 in self.human_collision_geoms) or 
                    (contact.geom2 in self.human_collision_geoms)):
                    print('Human-robot collision detected between ', self.sim.model.geom_id2name(contact.geom1), ' and ', self.sim.model.geom_id2name(contact.geom2))
                    geom2_body = self.sim.model.geom_bodyid[self.sim.data.contact[i].geom2]
                    print(' Contact force on geom2 body', self.sim.data.cfrc_ext[geom2_body])
                    print('norm', np.sqrt(np.sum(np.square(self.sim.data.cfrc_ext[geom2_body]))))
                    print("Speed of robot part:")
                    ### !! This value is not correct since it rapidely changes BEFORE the collision
                    # Ways to handle this:
                    # 1) forward dynamic of the robot
                    #   --> We need to do this anyway at some point to allow low speed driving
                    # 2) save velocity of the last few timesteps
                    #   It would be enough to only safe if the velocity of any robot part is larger than a threshold.
                    if contact.geom1 in self.robot_collision_geoms:
                        print(self.sim.data.geom_xvelp[contact.geom1])
                        print(self.sim.data.geom_xvelr[contact.geom1])
                    else:
                        print(self.sim.data.geom_xvelp[contact.geom2])
                        print(self.sim.data.geom_xvelr[contact.geom2])
                    stop=0
                else:
                    print('Collision with static environment detected between ', self.sim.model.geom_id2name(contact.geom1), ' and ', self.sim.model.geom_id2name(contact.geom2))
                    # There's more stuff in the data structure
                    # See the mujoco documentation for more info!
                    geom2_body = self.sim.model.geom_bodyid[self.sim.data.contact[i].geom2]
                    print(' Contact force on geom2 body', self.sim.data.cfrc_ext[geom2_body])
                    print('norm', np.sqrt(np.sum(np.square(self.sim.data.cfrc_ext[geom2_body]))))
        return self.sim.data.ncon > 0

    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        super()._load_model()
        # Adjust base pose accordingly
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        # load model for table top workspace
        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])

        # Modify default agentview camera
        mujoco_arena.set_camera(
            camera_name="agentview",
            pos=[0.0, -2, 2],
            quat=[-0.0705929, 0.0705929, 0.7035742, 0.7035742],
        )

        # Initialize human
        self.human = HumanObject(
            name="Human"
        )

        # Create placement initializer
        if self.placement_initializer is not None:
            self.placement_initializer.reset()
            self.placement_initializer.add_objects(self.human)
        else:
            self.placement_initializer = UniformRandomSampler(
                name="ObjectSampler",
                mujoco_objects=self.human,
                x_range=[-0.0, 0.0],
                y_range=[-0.0, 0.0],
                rotation=(0, 0),
                rotation_axis="x",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.0,
            )

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.human,
        )

    def _create_new_controller(self):
        """Manually override the controller with the failsafe controller."""
        if self.use_failsafe_controller:
            self.robots[0].controller_config["base_pos"] = self.robots[0].base_pos
            self.robots[0].controller_config["base_orientation"] = self.robots[0].base_ori
            self.robots[0].controller_config["control_sample_time"] = self.control_sample_time
            self.failsafe_controller = FailsafeController(**self.robots[0].controller_config)
        else:
            self.failsafe_controller = None

    def _override_controller(self):
        """Manually override the controller with the failsafe controller."""
        if self.failsafe_controller is not None:
            self.robots[0].controller = self.failsafe_controller

    def _set_human_measurement(self, human_measurement, time):
        """Set the human measurement in the failsafe controller.
        
        Args:
          human_measurement (list[list[double]]): List of human measurements [x, y, z]-joint positions.
              The order of joints is defined in `human.py`
          time (double): Current time
        """
        if self.failsafe_controller is not None:
            self.robots[0].controller.set_human_measurement(human_measurement, time)

    def _setup_references(self):
        """
        Sets up references to important components. A reference is typically an
        index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        super()._setup_references()
        if self.control_sample_time % self.model_timestep != 0:
          self.control_sample_time = math.floor(self.control_sample_time/self.model_timestep) * self.model_timestep

        simulation_step_freq = int(1/self.model_timestep)
        self.human_animation_step_length = simulation_step_freq/self.human_animation_freq
        assert self.human_animation_step_length >= 1, "No human animation frequency faster than {} Hz is allowed".format(self.model_freq)


    def _setup_observables(self):
        """
        Sets up observables to be used for this environment. Creates object-based observables if enabled

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()

        # low-level object information
        if self.use_object_obs:
            # Get robot prefix and define observables modality
            pf = self.robots[0].robot_model.naming_prefix
            modality = "object"

            @sensor(modality=modality)
            def gripper_pos(obs_cache):
                return (
                    obs_cache[f"{pf}eef_pos"] if f"{pf}eef_pos" in obs_cache else np.zeros(3)
                )

            sensors = [gripper_pos]
            names = [s.__name__ for s in sensors]

            # Create observables
            for name, s in zip(names, sensors):
                observables[name] = Observable(
                    name=name,
                    sensor=s,
                    sampling_rate=self.control_freq,
                )

        return observables


    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        super()._reset_internal()

        self._create_new_controller()
        self._override_controller()

        self.animation_start_time = 0
        self.low_level_time = 0
        self.animation_time = -1

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        if not self.deterministic_reset:

            # Sample from the placement initializer for all objects
            object_placements = self.placement_initializer.sample()

            # We know we're only setting a single object (the door), so specifically set its pose
            human_pos, human_quat, _ = object_placements[self.human.name]
            self.human_pos_offset = [self.base_human_pos_offset[i] + human_pos[i] for i in range(3)]
            # Loop through all objects and reset their positions
            #for obj_pos, obj_quat, obj in object_placements.values():
            #    self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))
        

    def _control_human(self):
        """
        Set the human joint positions according to the human animation files.
        """
        # Convert low level time to human animation time
        control_time = math.floor(self.low_level_time/self.human_animation_step_length)
        # If the animation time would stay the same, there is no need to update the human.
        if control_time - self.animation_start_time == self.animation_time:
          return
        self.animation_time = control_time - self.animation_start_time
        # Check if current animation is finished
        if self.animation_time > self.human_animations[self.human_animation_id]["Pelvis_pos_x"].shape[0]-1:
            # Rotate to next human animation
            self.human_animation_id = self.human_animation_id+1 if self.human_animation_id < len(self.human_animations)-2 else 0
            self.animation_time = 0
            self.animation_start_time = control_time
        
        ## Root bone transformation
        animation_pos = [self.human_animations[self.human_animation_id]["Pelvis_pos_x"][self.animation_time],
                         self.human_animations[self.human_animation_id]["Pelvis_pos_y"][self.animation_time],
                         self.human_animations[self.human_animation_id]["Pelvis_pos_z"][self.animation_time]]
        animation_offset = self.animation_info[self.human_animation_names[self.human_animation_id]]["position_offset"]
        # These settings are adjusted to fit the CMU motion capture BVH files!
        human_pos = [ (animation_pos[0] + self.human_pos_offset[0] + animation_offset[0]),
                      (animation_pos[1] + self.human_pos_offset[1] + animation_offset[1]),
                      (animation_pos[2] + self.human_pos_offset[2] + animation_offset[2])]
        # Base rotation (without animation)
        animation_offset_rot = Rotation.from_quat(self.animation_info[self.human_animation_names[self.human_animation_id]]["orientation_quat"])
        human_rot = self.human_base_quat.__mul__(animation_offset_rot)
        # Apply rotation to position
        human_pos = human_rot.apply(human_pos)
        # Animation rotation
        rot = Rotation.from_quat(self.human_animations[self.human_animation_id]["Pelvis_quat"][self.animation_time])
        human_rot = human_rot.__mul__(rot)
        human_quat = rot_to_quat(human_rot)
        # Set base position and rotation
        human_body_id = self.sim.model.body_name2id(self.human.root_body)
        self.sim.model.body_pos[human_body_id] = human_pos
        self.sim.model.body_quat[human_body_id] = human_quat
        # Set rotation of all other joints
        for joint_element in self.human.joint_elements:
            joint_name = joint_element + "_x"
            self.sim.data.set_joint_qpos(self.human.naming_prefix + joint_name, 
                self.human_animations[self.human_animation_id][joint_name][self.animation_time])
            joint_name = joint_element + "_y"
            self.sim.data.set_joint_qpos(self.human.naming_prefix + joint_name, 
                self.human_animations[self.human_animation_id][joint_name][self.animation_time])
            joint_name = joint_element + "_z"
            self.sim.data.set_joint_qpos(self.human.naming_prefix + joint_name, 
                self.human_animations[self.human_animation_id][joint_name][self.animation_time])

    def _human_measurement(self):
        """
        Retrieve the human measurements and save them to self.human_measurement.
        """ 
        self.human_measurement = [self.sim.data.get_site_xpos("Human_" + joint_element)
                                    for joint_element in self.human.joint_elements]


    def _visualize_reachable_sets(self):
        """Visualize the robot and human reachable set.
        """
        if self.use_failsafe_controller:
            robot_capsules = self.robots[0].controller.get_robot_capsules()
            for cap in robot_capsules:
                self.viewer.viewer.add_marker(pos=cap.pos, type=3, size=cap.size, mat=cap.mat.flatten(), rgba=[0.0, 0.0, 1.0, 0.2], label="", shininess=0.0)
            human_capsules = self.robots[0].controller.get_human_capsules()
            for cap in human_capsules:
                self.viewer.viewer.add_marker(pos=cap.pos, type=3, size=cap.size, mat=cap.mat.flatten(), rgba=[0.0, 1.0, 0.0, 0.2], label="", shininess=0.0)
            # Visualize human joints
            #for joint_element in self.human.joint_elements:
            #    pos = self.sim.data.get_site_xpos("Human_" + joint_element)
            #    self.viewer.viewer.add_marker(pos=pos, type=2, size=[0.05, 0.05, 0.05], mat=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], rgba=[1.0, 0.0, 0.0, 1.0], label="", shininess=0.0)
            


    @property
    def _visualizations(self):
        """
        Visualization keywords for this environment

        Returns:
            set: All components that can be individually visualized for this environment
        """
        vis_set = super()._visualizations
        return vis_set

    def visualize(self, vis_settings):
        """
        In addition to super call, visualize gripper site proportional to the distance to the cube.

        Args:
            vis_settings (dict): Visualization keywords mapped to T/F, determining whether that specific
                component should be visualized. Should have "grippers" keyword as well as any other relevant
                options specified.
        """
        # Run superclass method first
        super().visualize(vis_settings=vis_settings)
        #self.human.visualize(vis_settings=vis_settings)

        # Color the gripper visualization site according to its distance to the goal
        ## TODO
        #if vis_settings["grippers"]:
        #self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.cube)
