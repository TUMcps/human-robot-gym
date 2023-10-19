Define Your Own Environment
===========================

**Disclaimer: Please evaluate carefully if your functionality should be in a wrapper or in a new environment!**
See `the official gym documentation <https://www.gymlibrary.dev/content/environment_creation/>`_ for more details.

To define your own environment, you need to create a new class that inherits from the ``HumanEnv`` class or subclasses of that.
We want to explain the elements of a typical environment and where you might want to add your code.

.. _step-function:
Step function
-------------

The ``step`` function is the core of all ``gym`` environments.

.. code-block:: python

    observation, reward, done, info = step(action)

It executes the given action and returns the next observation, reward, whether the episode is finished and an information dictionary.
If you want any additional functionality that should be executed in every step, override the step function, e.g.:

.. code-block:: python

    def step(self, action):
        """Step the environment with my custom functionality.
        
        Adds my custom functionality to the step by doing some cool stuff.

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
        # Your code that you want to execute before the step here
        obs, reward, done, info = super().step(action)
        # Your code that you want to execute after the step here
        return obs, reward, done, info

.. _reset-function:
Reset Function
---------------

The ``reset`` function is another core function of all ``gym`` environments.
It resets the environment to its initial state and returns the initial observation.

.. code-block:: python

    observation = reset()

If you want to add any additional functionality to the reset function, you can override it, e.g.:

.. code-block:: python

    def reset(self):
        """Resets the environment with my custom functionality.
        
        Adds my custom functionality to the reset by doing some cool stuff.

        Returns:
            OrderedDict: observations from the environment
        """
        obs = super().reset()
        # Your code that you want to execute after the reset here
        return obs

For most internal reset functionality, we recommend the ``_reset_internal`` function:

.. code-block:: python

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        # Your code that you want to execute before the simulation is resetted
        super()._reset_internal()
        # Your code that you want to execute after the simulation is resetted


.. _get-info-function:
_get_info Function
------------------

The ``_get_info`` function is a method that is called by the environment's ``step`` function.
It returns a dictionary containing information about the current step, such as whether there was a collision, whether a timeout was reached, and whether the failsafe controller intervened.

.. code-block:: python

    def _get_info(self) -> Dict:
        """Return the info dictionary of this step.

        Returns
            info dict containing of
                * collision: if there was a collision or not
                * collision_type: type of collision
                * timeout: if timeout was reached
                * failsafe_intervention: if the failsafe controller intervened
                    in this step or not
        """
        info = super()._get_info()
        # Add more info if wanted (do not forget to pass this to the tensorboard callback)
        # info["my_cool_info"] = 0
        return info

If you want to add more information to the info dictionary, you can override this function and add your own information, as shown in the example above. Note that if you do add your own information, you will need to make sure to pass it to the tensorboard callback in order for it to be logged.

It is important to note that this function is not meant to be called directly by the agent. Instead, it is called internally by the environment's ``step`` function.

.. _reward-done-function:
Reward and Done Calculation
----------------------------

The fundamentals
^^^^^^^^^^^^^^^^
With each environment, you have to define the ``_check_success`` function.
See ``reach_human_env.py`` for an example implementation.

.. code-block:: python

    def _check_success(
        self, achieved_goal: List[float], desired_goal: List[float]
    ) -> bool:
        """Check if the desired goal was reached.

        Should be overridden by subclasses to specify task success conditions.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
        Returns:
            True if success
        """
        return False

You also have to define, where the achieved and desired goal can be found in the observation using:

.. code-block:: python

    def _get_achieved_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """
        Extract the achieved goal from the observation.

        The achieved goal is the new joint angle position of all joints.

        Args:
            observation: The observation after the action is executed

        Returns:
            The achieved goal
        """
        return observation["my_obs"]
    
    def _get_desired_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """Extract the desired goal from the observation.

        The desired goal is a desired goal joint position.

        Args:
            observation: The observation after the action is executed

        Returns:
            The desired goal
        """
        return observation["desired_goal"]

The ``_check_done`` function determines whether an episode should be terminated.
By default, this function returns ``True`` either on task success or if an illegal collision has occurred (collision with the static environment, self-collisions or collisions with the human above a velocity threshold).
These termination conditions can be deactivated individually by setting ``done_at_success`` or ``done_at_collision`` to ``False``. 

.. code-block:: python

    def _check_done(
        self, achieved_goal: List[float], desired_goal: List[float], info: Dict
    ) -> bool:
        """Compute the done flag based on the achieved goal, the desired goal, and the info dict.

        This function can only be called for one sample.

        Returns `done=True` if either
            - the desired goal was reached and `self.done_at_success=True`
            - a collision occurred and `self.done_at_collision=True`

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision
        Returns:
            done
        """
        if self.done_at_collision and self._check_illegal_collision(COLLISION_TYPE(info["collision_type"])):
            return True

        if self.done_at_success and self._check_success(achieved_goal, desired_goal):
            return True
        return False

In the ``reward`` function, the reward is calculated based on the achieved goal, desired goal, and the info dictionary.
It distributes the reward calculation to the ``_sparse_reward``, ``_dense_reward``, and ``_collision_reward`` functions, that may be overridden by subclasses.
Additionally, it employs a constant reward scaling factor, if specified in ``reward_scaling``.

.. code-block:: python

    def reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute the reward based on the achieved goal, the desired goal, and the info dict.

        If `self.reward_shaping`, we use a dense reward, otherwise a sparse reward.
        The sparse reward yields
            - `self.task_reward` if the target is reached
            - `self.object_gripped_reward` if the object is gripped but the target is not reached
            - `-1` otherwise

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions
        Returns:
            float: reward
        """
        reward = self._sparse_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        if self.reward_shaping:
            reward += 1 + self._dense_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        # Add a penalty for self-collisions and collisions with the human
        collision_reward = self._collision_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        reward = reward + collision_reward

        # Scale reward if requested
        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

By default, the sparse reward yields ``task_reward`` (which can be controlled via a environment parameter) on task success and ``-1`` otherwise.

.. code-block:: python

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a sparse reward based on the achieved goal, the desired goal, and the info dict.

        The sparse reward function yields
            - `self.task_reward` if the target is reached,
            - `-1` otherwise.

        This method may be overridden by subclasses to add subgoal rewards.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: sparse environment reward
        """
        if self.goal_reached:
            return self.task_reward
        else:
            return -1

The dense reward calculation has to be defined in subclasses.

.. code-block:: python

    def _dense_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a dense guidance reward based on the achieved goal, the desired goal, and the info dict.

        This method may be overridden to add environment-specific dense rewards.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: dense environment reward
        """
        return 0.0

The collision reward yields a constant reward of ``collision_reward`` if an illegal collision occurs.

.. code-block:: python

    def _collision_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a penalty for self-collisions, collisions with the static environment,
        and critical collisions with the human.

        Collisions that are not critical, or that involve white-listed objects are not penalized

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: collision penalty
        """
        if self._check_illegal_collision(COLLISION_TYPE(info["collision_type"])):
            return self.collision_reward
        else:
            return 0.0


Hindsight Experience Replay
^^^^^^^^^^^^^^^^^^^^^^^^^^^
We use a custom version of reward calculation that allows us to use `hindsight experience replay <https://stable-baselines3.readthedocs.io/en/master/modules/her.html>`_ if we would like.
The reward and done flag are calculated in the environment's ``step`` function based on the current state of the environment and the agent's action. The following code shows how the reward and done flag are calculated:

.. code-block:: python

    achieved_goal = self._get_achieved_goal_from_obs(observations)
    desired_goal = self._get_desired_goal_from_obs(observations)
    self.goal_reached = self._check_success(
        achieved_goal=achieved_goal,
        desired_goal=desired_goal
        )
    if self.goal_reached:
        self.n_goal_reached += 1
    info = self._get_info()
    reward = self._compute_reward(
        achieved_goal=achieved_goal,
        desired_goal=desired_goal,
        info=info
        )
    done = self._compute_done(
        achieved_goal=achieved_goal,
        desired_goal=desired_goal,
        info=info
        )

The achieved and desired goals are obtained from the observations using the ``_get_achieved_goal_from_obs`` and ``_get_desired_goal_from_obs`` functions, respectively.
The environment keeps track of whether the goal has been reached using the ``goal_reached`` attribute, which is updated in the ``_check_success`` function.

If the goal is reached, the ``n_goal_reached`` attribute is incremented to keep track of how many times the agent has reached the goal. 
The ``_get_info`` function is called to obtain any additional information about the current step, such as whether there was a collision or if a timeout was reached.

The reward is computed using the ``_compute_reward`` function, which takes in the achieved goal, desired goal, and any additional information obtained from the ``_get_info`` function.

.. code-block:: python

    def _compute_reward(
        self,
        achieved_goal: Union[List[float], List[List[float]]],
        desired_goal: Union[List[float], List[List[float]]],
        info: Union[Dict, List[Dict]],
    ) -> Union[float, List[float]]:
        """Compute the reward based on the achieved goal, the desired goal, and the info dict.

        This function can either be called for one sample or a list of samples.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision

        Returns:
            reward (list of rewards)
        """
        if isinstance(info, Dict):
            # Only one sample
            return self.reward(achieved_goal, desired_goal, info)
        else:
            rewards = [
                self.reward(a_g, d_g, i)
                for (a_g, d_g, i) in zip(achieved_goal, desired_goal, info)
            ]
            return rewards

As you can see, this calls the ``reward`` function that you have to define yourself!
The same is done for the ``done`` flag.

.. _setup-function:
_setup_arena Function
---------------------

If you want to change the objects in the scene, you have to adapt the ``_setup_arena`` function.
Here is an example:

.. code-block:: python 

        def _setup_arena(self):
            """Set up the mujoco arena.

            Must define self.mujoco_arena.
            Define self.objects and self.obstacles here.
            """
            # load model for table top workspace
            self.mujoco_arena = TableArena(
                table_full_size=self.table_full_size,
                table_offset=self.table_offset,
                xml=xml_path_completion("arenas/table_arena.xml")
            )

            # Arena always gets set to zero origin
            self._set_origin()

            # Modify default agentview camera
            self._set_mujoco_camera()

            # << OBJECTS >>
            # Objects are elements that can be moved around and manipulated.
            # Create objects
            # Box example
            box_size = np.array([0.05, 0.05, 0.05])
            box = BoxObject(
                name="smallBox",
                size=box_size * 0.5,
                rgba=[0.1, 0.7, 0.3, 1],
            )
            self.objects = [box]
            # Placement sampler for objects
            bin_x_half = self.table_full_size[0] / 2 - 0.05
            bin_y_half = self.table_full_size[1] / 2 - 0.05
            self.object_placement_initializer = self._setup_placement_initializer(
                name="ObjectSampler",
                initializer=self.object_placement_initializer,
                objects=self.objects,
                x_range=[-bin_x_half, bin_x_half],
                y_range=[-bin_y_half, bin_y_half],
            )
            # << OBSTACLES >>
            self._setup_collision_objects(
                add_table=True,
                add_base=True,
                safety_margin=0.01
            )
            # Obstacles are elements that the robot should avoid.
            self.obstacles = []
            self.obstacle_placement_initializer = self._setup_placement_initializer(
                name="ObstacleSampler",
                initializer=self.obstacle_placement_initializer,
                objects=self.obstacles,
            )


.. _observables:
Observables
-----------

Observables define all things that can possibly be observed, that doesn't mean they necessarly are in the observation that the agent receives from the step function.
The observables are defined in the ``_setup_observables`` function.

.. code-block:: python

    def _setup_observables(self):
        """Set up observables to be used for this environment.

        Creates object-based observables if enabled.

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()
        # Here you can activate and deactivate observables so that they appear in the agent's observation
        # E.g. activate joint position observation.
        prefix = self.robots[0].robot_model.naming_prefix
        if prefix + "joint_pos" in observables:
            observables[prefix + "joint_pos"].set_active(True)
        # Or deactivate the human joint position measurements
        if "human_joint_pos" in observables:
            observables["human_joint_pos"].set_active(False)
        
        # You can add new observables like this:
        # First define the type of observation
        modality = "goal"

        # Create a "sensor" that observes something
        @sensor(modality=modality)
        def goal_difference(obs_cache):
            return self.desired_goal - np.array([self.sim.data.qpos[x] for x in self.robots[0]._ref_joint_pos_indexes])
        
        # Create a list of all sensors
        sensors = [goal_difference]

        # Finish up observables
        names = [s.__name__ for s in sensors]
        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )
        return observables

You can then define which types of observations the agent should receive in the ``GymWrapper`` (see :doc:`demos section</demos>` for an example!).

Human Animations
----------------

As described in the :doc:`human animation section</human_animations>`, ``HumanEnv`` environments feature an animated human that follows real-world motion capturing data.
The ``_control_human`` method is responsible for this behavior.
To determine the progress in the animation, it calls the ``_compute_animation_time`` method, which returns the intex of the current frame in the animation.
This method may be overridden to transform the progress in the animation, e.g. to loop the animation by alternating between forward and backward replay.
For more information, please refer to the :doc:`human animation section</human_animations>`.
Linear playback is implemented as

.. code-block:: python

    def _compute_animation_time(self, control_time: float) -> float:
        """Compute the animation time from the control time and the animation start time.

        Can be overridden to implement custom animation time computation, e.g. for looping, pausing, etc.

        Args:
            control_time (float): Current control time

        Returns:
            float: Animation time
        """
        return control_time - self.animation_start_time

Once an animation is finished the ``_progress_to_next_animation`` method is called from within ``_control_human`` to determine the next animation.
We sample a list of animations at the beginning of each episode in ``_reset_internal`` and then cycle through them.
This is vital for SIR to be able to replay episodes from datasets.

.. code-block:: python

    def _progress_to_next_animation(self, animation_start_time: int):
        """Changes the human animation id during an episode.

        Args:
            animation_start_time (int): Current control time. Used to set the animation start time.
        """
        self._human_animation_ids_index = (
            (self._human_animation_ids_index + 1) % self._n_animations_to_sample_at_resets
        )
        self.animation_time = 0
        self.animation_start_time = animation_start_time
