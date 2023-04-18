Environment wrappers
====================

Environment wrappers add functionality to the environment.
Everything that is optional, and not part of the core environment should be defined as an environment wrapper.

Available wrappers
------------------

In addition to the usual gym wrappers, the following wrappers are available:
    - ``CollisionPreventionWrapper``: Prevents the agent from colliding with the static environment and itself.
    - ``CartActionBasedExpertImitationRewardWrapper``: Adds a reward based on the distance to the expert trajectory.
    - ``ExpertObsWrapper``: Allows to assemble observations for scripted expert policies.
    - ``GoalEnvironmentGymWrapper``: Allows to train using hindsight experience replay (HER) by adding the goal state to the observation.
    - ``IKPositionDeltaWrapper``: Allows to use Cartesian position deltas as actions.
    - ``TimeLimit``: Limits the number of steps per episode.
    - ``VisualizationWrapper``: Allows to visualize the environment.


Writing your own wrapper
------------------------

Wrappers usually override the ``step()`` and ``reset()`` methods.
You can take a look at the ``IKPositionDeltaWrapper`` for an example.