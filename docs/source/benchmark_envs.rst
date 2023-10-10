Benchmark Environments
======================

The **human-robot-gym** features eight unique environments, which are outlined in this section.

General Remarks
---------------

Tasks and Animations
^^^^^^^^^^^^^^^^^^^^

The environments in the **human-robot-gym** can be divided into two groups, depending on the degree of interaction between the robot and the human:

    - **Coexistence**: The human and the robot perform separate tasks nearby. The environments that belong to this group are:

        - **HumanReach** and
        - **PickPlaceHuman**.

    - **Collaboration**: The human and the robot perform a task together. To this group, the following environments belong:

        - **ObjectInspection**,
        - **RobotHumanHandover**,
        - **HumanRobotHandover**,
        - **CollaborativeLifting**,
        - **CollaborativeHammering**, and
        - **CollaborativeStacking**.

A characteristic of the latter group is that these environments **tie task completion to finishing animation clips of the human**.
Therefore, tasks are considered successful once the human has finished its animation clip, and the animation can only finish if the robot productively interacts with the human.
In contrast, in the coexistence environments, multiple clips may be played back until a task is considered successful, or the robot may finish the task before the first clip is finished.
The animations we provide fir the collaboration tasks contain a **retreat phase** at the end, where the human moves away from the table to a starting position.
This has three core effects:

    - animations may be **chained together seamlessly** as the human starts and ends their motion at the same point, while
    - there is no possibility for collisions between the human and the robot at the start of the episode. Meanwhile,
    - **task rewards are given with a delay** as the main challenges in the tasks are generally already solved when the human retreats.

However, when recording custom animations, including a retreat phase is not necessary.


In all environments, it is possible to either terminate an episode when a task is considered successful, or to continue the episode by sampling a new task.
Which of these options is used can be specified by adjusting the ``done_at_success`` parameter.
In either case, episodes are terminated after a maximum number of steps, which is specified by the ``horizon`` parameter.

Rewards
^^^^^^^

The environment rewards :math:`r^{env}` follow a fixed scheme in all environments. The reward function is defined as follows:

.. math::
    r^{env}(s, a) = \lambda \cdot (r^{coll}(s, a) + \hat{r}^{\,env}(s, a))\,,

where :math:`\lambda` is a reward scaling factor (``reward_scale`` in the code), :math:`r^{coll}` is a reward penalty for collisions (``collision_reward`` in the code), and :math:`\hat{r}^{env}` is the environment-specific reward, given by

.. math::
    \hat{r}^{\,env}(s, a) = \begin{cases} \hat{r}^{\,env,sparse}(s, a) + 1 + \hat{r}^{\,env,dense}(s, a)\,, & \text{if } \texttt{reward_shaping} \\ \hat{r}^{\,env,sparse}(s, a)\,, & \text{else} \end{cases}\,.

``reward_shaping`` is a Boolean environment parameter that determines whether a **dense guidance reward** :math:`\hat{r}^{\,env,dense}` should be added to the **sparse reward** :math:`\hat{r}^{\,env,sparse}`.
The latter follows the scheme

.. math::
    \hat{r}^{\,env,sparse}(s, a) = \begin{cases} \hat{r}^{\,env,task}\,, & \text{if task complete} \\ \vdots \\ -1\,, & \text{else} \end{cases}\,,


where :math:`\hat{r}^{\,env,task}` is a constant task success reward that is referred to as ``task_reward`` in the code. The task completion conditions are defined by the environment.
The enviroments also may provide **subgoal rewards** that add more cases to the definition of :math:`\hat{r}^{\,env,sparse}`.
The dense rewards :math:`\hat{r}^{\,env,dense}` are defined individually by the environments.


Environment Descriptions
------------------------

HumanReach
^^^^^^^^^^

**Two variants** exist of this task, one for **IK** and one for **joint space control**. These are represented by the ``ReachHuman`` and ``ReachHumanCart`` classes, respectively.

Behavior of the Human
"""""""""""""""""""""

The human performs separate tasks nearby (**coexistence** with the robot).
By default, the human is animated using motion capturing clips from the `CMU motion capture data set <http://mocap.cs.cmu.edu/>`_.

Goal
""""

    - In the **IK control** case: reach a target position with the robot end-effector.
    - With **joint space control**: reach a target joint configuration.

Reward
""""""

The **HumanReach** environment **does not provide subobjective rewards**. Therefore, the **sparse reward** in the **HumanReach** environment :math:`\hat{r}^{\,env,sparse,HR}` is defined as

.. math::
    \hat{r}^{\,env,sparse,HR}(s, a) = \begin{cases} \hat{r}^{\,env,task,HR}\,, & \text{if task complete} \\ -1\,, & \text{else} \end{cases}\,. 

As described above, :math:`\hat{r}^{\,env,task,HR}` is a constant task success reward that is referred to as ``task_reward`` in the code.

The **dense reward** :math:`\hat{r}^{env,dense,HR}` is defined as

.. math::
    \hat{r}^{\,env,dense,HR}(s, a) = -0.1 \cdot \|\vec{v}^{\,tar}\|\,,

where :math:`\vec{v}^{\,tar}` is the vector between the current and target positions. Depending on the action space, this vector is either in joint space (joint space control) or in Cartesian space (IK control).


PickPlaceHuman
^^^^^^^^^^^^^^

Our pick-and-place task requires the robot to transfer a small cube-shaped object from one location to another.
The environment is represented by the ``PickPlaceHumanCart`` class.
The environments

    - **ObjectInspection**,
    - **RobotHumanHandover**, and
    - **HumanRobotHandover**

inherit from this class.


Behavior of the Human
"""""""""""""""""""""

The human performs separate tasks nearby (**coexistence** with the robot).
By default, the human is animated using motion capturing clips from the `CMU motion capture data set <http://mocap.cs.cmu.edu/>`_.

Goal
""""

The goal of the task is to pick up a small cube-shaped object from a table and place it near a target location.
The task is successful, if the object is placed within a certain radius ``goal_dist`` around the target location.

Reward
""""""

The **PickPlaceHuman** environment provides the option to add a reward for the **robot grasping the object**. Thus, the sparse reward is given by

.. math::
    \hat{r}^{\,env,sparse,PP} = \begin{cases} \hat{r}^{\,env,task, PP}\, & \text{if task complete} \\ \hat{r}^{\,env,subgoal,PP,grasp}\,, & \text{else if object grasped} \\ -1\,, & \text{else} \end{cases}\,,

where :math:`\hat{r}^{\,env,task,PP}` and :math:`\hat{r}^{\,env,subgoal,PP,grasp}` are constant rewards that are referred to as ``task_reward`` and ``object_gripped_reward`` in the code.
By default, the latter is deactivated (``object_gripped_reward = -1``).
The dense reward :math:`\hat{r}^{\,env,dense,PP}` is defined as

.. math::
    \hat{r}^{\,env,dense,PP}(s, a) = -0.1 \cdot (\|\vec{v}^{\,obj}\| \cdot 0.2 + \|\vec{v}^{\,tar}\|)\,,

where :math:`\vec{v}^{\,obj}` is the vector between the end-effector and object positions, and :math:`\vec{v}^{\,tar}` is the vector between the end-effector and target positions.
Both vectors are in Cartesian space.


ObjectInspection
^^^^^^^^^^^^^^^^

This environment represens a lifting task, where the robot lifts a small cube-shaped object from a table and moves it in front of the human's head.
When this has been achieved, the object should remain in front of the human's face for long enough to allow the human to inspect it from different sides.
This task is implemented in the ``HumanObjectInspectionCart`` class. It inherits from the ``PickPlaceHumanCart`` class.

Behavior of the Human
"""""""""""""""""""""

The human approaches the table and waits until the object arrives within a certain radius ``goal_dist`` around a point in front of the human's head.
The exact location of this point with respect to the human is set in the :doc:`animation info files</human_animations>`.
When the object is within this radius, the human starts to inspect the object from different sides.
If the object is moved out of the target zone, the human returns to the wait phase and the inspection has to be restarted.
If the inspection is complete, the human walks away from the table to a starting position.

Goal
""""

This collaboration task is **tied to the human animations**.
Thus, the task is successful if the animation clip of the human is finished.
The animations we provide feature a **retreat phase** at the end, where the human moves away from the table.
The behavior of the human in this phase is not relevant for the task success. However, the task reward is only given when this phase is completed.

Reward
""""""

The sparse reward :math:`\hat{r}^{\,env,sparse,OI}` in the **ObjectInspection** task provides two different subobjective rewards

    - for the **robot grasping the object** (``object_gripped_reward``), and
    - for the **object being within the target zone** (``object_at_target_reward``),

and is formulated as

.. math::
    \hat{r}^{\,env,sparse,OI}(s, a) = \begin{cases} \hat{r}^{\,env,task,OI}\, & \text{if task complete} \\ \hat{r}^{\,env,subgoal,OI,target}\, & \text{else if object at target} \\ \hat{r}^{\,env,subgoal,OI,grasp}\,, & \text{else if object grasped} \\ -1\,, & \text{else} \end{cases}\,,

:math:`\hat{r}^{\,env,task,OI}`, :math:`r^{\,env,subgoal,OI,grasp}`, and :math:`r^{\,env,subgoal,OI,target}` are constant rewards that are referred to as ``task_reward``, ``object_gripped_reward``, and ``object_at_target_reward`` in the code, respectively.
By default, the subgoal rewards are deactivated (``object_gripped_reward = -1`` and ``object_at_target_reward = -1``).

The dense reward :math:`\hat{r}^{\,env,dense,OI}` is defined analogously to the **PickPlaceHuman** environment as

.. math::
    \hat{r}^{\,env,dense,OI}(s, a) = -0.1 \cdot (\|\vec{v}^{\,obj}\| \cdot 0.2 + \|\vec{v}^{\,tar}\|)\,,

where :math:`\vec{v}^{\,obj}` is the vector between the end-effector and object positions, and :math:`\vec{v}^{\,tar}` is the vector between the end-effector and target positions.


RobotHumanHandover
^^^^^^^^^^^^^^^^^^

The **RobotHumanHandover** environment requires the robot to hand over a hammer to the human.
Initially, the hammer is placed on the table in front of the robot.
The robot has to pick it up and place it into the palm of a hand that the human holds out over the table.
The ``RobotHumanHandoverCart`` class implements this task.
It inherits from the ``PickPlaceHumanCart`` class.

Behavior of the Human
"""""""""""""""""""""

The human approaches the table and extends one arm over the table.
Into which hand the hammer should be placed is specified in the :doc:`animation info files</human_animations>`.
In this posture, they wait until the collision geometries of the hammer intersect with a ellipsoid at the palm which the human holds out.
Once this is the case, a `weld equality <https://mujoco.readthedocs.io/en/2.1.2/XMLreference.html#equality-weld>`_ is activated, attaching the hammer to the human's hand.
Finally, the human returns to their starting position by moving away from the table.
Additionally to nine default animations, we also provide six animations with an increased difficulty, where the human uses the hammer to work at the table once they have received it or where they perform different tasks before receiving the hammer.

Goal
""""

This collaboration task is **tied to the human animations**.
Thus, the task is successful if the animation clip of the human is finished.
The animations we provide feature a **retreat phase** at the end, where the human moves away from the table.
The behavior of the human in this phase is not relevant for the task success.
However, the task reward is only given when this phase is completed.

Reward
""""""

The sparse reward :math:`\hat{r}^{\,env,sparse,RHH}` in the **RobotHumanHandover** task provides two optional subobjective rewards

    - for the **robot grasping the object** (``object_gripped_reward``), and
    - for the **human holding the object** (``object_in_human_hand_reward``),

and is formulated as

.. math::
    \hat{r}^{\,env,sparse,RHH}(s, a) = \begin{cases} \hat{r}^{\,env,task,RHH}\, & \text{if task complete} \\ \hat{r}^{\,env,subgoal,RHH,hand}\,, & \text{else if object in human hand} \\ \hat{r}^{\,env,subgoal,RHH,grasp}\, & \text{else if object grasped by robot} \\ -1\,, & \text{else} \end{cases}\,,

:math:`\hat{r}^{\,env,task,RHH}`, :math:`r^{\,env,subgoal,RHH,grasp}`, and :math:`r^{\,env,subgoal,RHH,hand}` are constant rewards that are referred to as ``task_reward``, ``object_gripped_reward``, and ``object_in_human_hand_reward`` in the code, respectively.
By default, the subgoal rewards are deactivated (``object_gripped_reward = -1`` and ``object_in_human_hand_reward = -1``).

The dense reward :math:`\hat{r}^{\,env,dense,RHH}` is defined analogously to the **PickPlaceHuman** environment as

.. math::
    \hat{r}^{\,env,dense,RHH}(s, a) = -0.1 \cdot (\|\vec{v}^{\,obj}\| \cdot 0.2 + \|\vec{v}^{\,tar}\|)\,,

where :math:`\vec{v}^{\,obj}` is the vector between the end-effector and object positions, and :math:`\vec{v}^{\,tar}` is the vector between the end-effector and target positions.


HumanRobotHandover
^^^^^^^^^^^^^^^^^^

In this task, the robot should receive a hammer from the human and put it to a target location.
The ``HumanRobotHandoverCart`` class implements this task, which is derived from the ``PickPlaceHumanCart`` class.

Behavior of the Human
"""""""""""""""""""""

In the beginning of each episode, the human approaches the table with the hammer in one hand.
Which hand is used is specified in the :doc:`animation info files</human_animations>`.
The human presents the hammer to the robot by extending their arm over the table.
In this posture, the human remains until the robot manages to grasp the hammer.

Afterwards, the human lowers the hand but remains at the table waiting for the robot to place the hammer at a target location.
Once this is done, the human returns to their starting position by moving away from the table.
Additionally to eight default animations, we provide three animations with an increased difficulty level.
Here, the human uses the hammer to work at the table before handing it over to the robot.

Goal
""""

The **HumanRobotHandover** task is **tied to the human animations**.
Thus, the task is successful if the animation clip of the human is finished.
The animations we provide feature a **retreat phase** at the end, where the human moves away from the table.
The behavior of the human in this phase is not relevant for the task success.
However, the task reward is only given when this phase is completed.

Reward
""""""

The sparse reward :math:`\hat{r}^{\,env,sparse,RHH}` in the **HumanRobotHandover** task provides two optional subobjective rewards

    - for the **robot grasping the object** (``object_gripped_reward``), and
    - for the **object being at the target location** (``object_at_target_reward``),

and is therefore formulated as

.. math::
    \hat{r}^{\,env,sparse,HRH}(s, a) = \begin{cases} \hat{r}^{\,env,task,HRH}\, & \text{if task complete} \\ \hat{r}^{\,env,subgoal,HRH,target}\,, & \text{else if object at target} \\ \hat{r}^{\,env,subgoal,HRH,grasp}\, & \text{else if object grasped by robot} \\ -1\,, & \text{else} \end{cases}\,,

:math:`\hat{r}^{\,env,task,HRH}`, :math:`r^{\,env,subgoal,HRH,grasp}`, and :math:`r^{\,env,subgoal,HRH,target}` are constant rewards that are referred to as ``task_reward``, ``object_gripped_reward``, and ``object_at_target_reward`` in the code, respectively.
By default, the subgoal rewards are deactivated (``object_gripped_reward = -1`` and ``object_at_target_reward = -1``).

The dense reward :math:`\hat{r}^{\,env,dense,HRH}` is defined analogously to the **PickPlaceHuman** environment as

.. math::
    \hat{r}^{\,env,dense,HRH}(s, a) = -0.1 \cdot (\|\vec{v}^{\,obj}\| \cdot 0.2 + \|\vec{v}^{\,tar}\|)\,,

where :math:`\vec{v}^{\,obj}` is the vector between the end-effector and object positions, and :math:`\vec{v}^{\,tar}` is the vector between the end-effector and target positions.


CollaborativeLifting
^^^^^^^^^^^^^^^^^^^^

In this task, the robot should lift a heavy board object together with the human.
The human controls the motion and the robot should move accordingly to keep the object in a horizontal position.
The ``CollaborativeLiftingCart`` class implements this task. It inherits directly from the ``HumanEnv`` class.

Behavior of the Human
"""""""""""""""""""""

The human lifts the board to certain heights.
The animation is played back linearly, such that adding new animations does not require defining keyframes or loop amplitudes as specified in :doc:`this section</human_animations>`.

Goal
""""

The **CollaborativeLifting** task is **tied to the human animations**.
However, in contrast to most other **human-robot-gym** environments, this environment includes **failure conditions**.
If the board is tilted too far, or the board is pulled out of the robot's gripper, the episode is terminated.


Reward
""""""

The sparse reward in the **CollaborativeLifting** task is structured differently from the other environments in that the base reward is positive.
Therefore, the goal is not to finish the the task as quickly as possible but to keep the board in balance as long as possible.
This task does not provide subobjective rewards, but offers failure rewards for

    - the **board being tilted too far** (``imbalance_failure_reward``), and
    - the **board being pulled out of the robot's gripper** (``board_released_reward``).

The sparse reward is defined as

.. math::
    \hat{r}^{\,env,sparse,CL}(s, a) = \begin{cases} \hat{r}^{\,env,task,CL}\, & \text{if task complete} \\ \hat{r}^{\,env,failure,CL,imbalance}\,, & \text{else if board tilted too far} \\ \hat{r}^{\,env,failure,CL,release}\,, & \text{else if board released} \\ 1\,, & \text{else} \end{cases}\,,

where :math:`\hat{r}^{\,env,task,CL}`, :math:`\hat{r}^{\,env,failure,CL,imbalance}`, and :math:`\hat{r}^{\,env,failure,CL,release}` are constant rewards that are referred to as ``task_reward``, ``imbalance_failure_reward``, and ``board_released_reward`` in the code, respectively.

The dense reward :math:`\hat{r}^{\,env,dense,CL}` incorporates the angle between the board and the horizontal plane and is given by

.. math::
    \hat{r}^{\,env,dense,CL}(s, a) = \frac{2}{\pi} \cdot \frac{sin^{-1}(\vec{n}^{board} \circ \vec{n}^{world}) - \phi^{min}}{\frac{\pi}{2} - \phi^{min}} - 2\,,

where :math:`\vec{n}^{board}` is the normal vector of the board, :math:`\vec{n}^{world}` is the normal vector of the horizontal plane, and :math:`\phi^{min}` is a minimum angle between the board and the horizontal plane.
In the code, :math:`sin(\phi^{min}`) can be controlled as ``min_balance``. We subtract :math:`2` from the reward to compensate for the default reward of :math:`1` (as opposed to :math:`-1` in the other environments).

CollaborativeHammering
^^^^^^^^^^^^^^^^^^^^^^

In the **CollaborativeHammering** environment, the robot should push a nail into a board that is held by the human.
The robot has a hammer in the gripper from the beginning of the episode.
The task is implemented in the ``CollaborativeHammeringCart`` class, which inherits from the ``HumanEnv`` class.

Behavior of the Human
"""""""""""""""""""""

The human approaches the table with the board in both hands and lay it onto the table.
Afterwards the human waits until the nail is pushed in sufficiently far.
Then, the human lifts the board again and moves away from the table to a starting position. 

Goal
""""

This collaboration task is **tied to the human animations**.
Thus, the task is successful if the animation clip of the human is finished.
The animations we provide feature a **retreat phase** at the end, where the human moves away from the table.
The behavior of the human in this phase is not relevant for the task success.

Reward
""""""

The sparse reward :math:`\hat{r}^{\,env,sparse,CH}` in the **CollaborativeHammering** task provides optional subobjective reward for the **hammer being driven far enough into the board** (``nail_hammered_in_reward``).
As a result, it is formulated as

.. math::
    \hat{r}^{\,env,sparse,CH}(s, a) = \begin{cases} \hat{r}^{\,env,task,CH}\, & \text{if task complete} \\ \hat{r}^{\,env,subgoal,CH}(s, a)\,, & \text{else} \end{cases}\,,

where

.. math::
    \hat{r}^{\,env,subgoal,CH}(s, a) = \begin{cases} \hat{r}^{\,env,subgoal,CH,nail}\,, & \text{if nail is hammered in} \\ -1\,, & \text{else} \end{cases} \quad + \\ \begin{cases} \hat{r}^{\,env,subgoal,CH,grasp}\,, & \text{if hammer grasped by robot} \\ 0\,, & \text{else} \end{cases}\,,

and :math:`\hat{r}^{\,env,task,CH}`, :math:`r^{\,env,subgoal,CH,nail}`, and :math:`r^{\,env,subgoal,CS,grasp}` are constant rewards that are referred to as ``task_reward``, ``nail_hammered_in_reward``, and ``hammer_gripped_reward_bonus`` in the code, respectively.
By default, subgoal rewards are deactivated (``nail_hammered_in_reward = -1`` and ``hammer_gripped_reward_bonus = 0``).


CollaborativeStacking
^^^^^^^^^^^^^^^^^^^^^

In this task, the robot and the human should assemble a stack of four cubes by alternatingly placing the cubes on top of each other.
The first cube is placed by the human.
The task is implemented in the ``CollaborativeStackingCart`` class, which inherits from the ``HumanEnv`` class.

Behavior of the Human
"""""""""""""""""""""

The human approaches the table with the cubes in both hands and places one of them onto the table.
Then, the human waits until the robot has placed the second cube on top of the first one.
Afterwards, the human places the third cube on top of the stack and waits again until the robot has placed the fourth cube on top of the stack.
Finally, the human moves away from the table to a starting position.

Goal
""""

This collaboration task is **tied to the human animations**.
Thus, the task is successful if the animation clip of the human is finished.
The animations we provide feature a **retreat phase** at the end, where the human moves away from the table.
The behavior of the human in this phase is not relevant for the task success.
However, unlike most of the **human-robot-gym** environments, the **CollaborativeStacking** environment includes a **failure condition**.
If the stack of cubes falls over, the episode is terminated.

Reward
""""""

The sparse reward :math:`\hat{r}^{\,env,sparse,CS}` in the **CollaborativeStacking** task provides optional subobjective reward **for each block the robot managed to add to the stack**.
Additionally, a failure reward is given **if the stack falls over**.

As such, the sparse reward is defined as

.. math::
    \hat{r}^{\,env,sparse,CS}(s, a) = \begin{cases} \hat{r}^{\,env,task,CS}\, & \text{if task complete} \\ \hat{r}^{\,env,failure,CS,toppled}\,, & \text{else if stack was toppled} \\ \hat{r}^{\,env,subgoal,CS}(s, a)\,, & \text{else} \end{cases}\,,

where

.. math::
    \hat{r}^{\,env,subgoal,CS}(s, a) = \begin{cases} \hat{r}^{\,env,subgoal,CS,A}\,, & \text{if robot added one cube to the stack} \\ \hat{r}^{\,env,subgoal,CS,B}\,, & \text{if robot added two cubes to the stack} \\ -1\,, & \text{else} \end{cases} \quad + \\ \begin{cases} \hat{r}^{\,env,subgoal,CS,grasp}\,, & \text{if object grasped by robot} \\ 0\,, & \text{else} \end{cases}\,,

and :math:`\hat{r}^{\,env,task,CS}`, :math:`\hat{r}^{\,env,failure,CS,toppled}`, :math:`r^{\,env,subgoal,CS,A}`, :math:`r^{\,env,subgoal,CS,B}`, and :math:`r^{\,env,subgoal,CS,grasp}` are constant rewards that are referred to as ``task_reward``, ``stack_toppled_reward``, ``second_cube_at_target_reward``, ``fourth_cube_at_target_reward``, and ``object_gripped_reward`` in the code, respectively.

The collaborative stacking environment does not yet provide a dense reward function.
