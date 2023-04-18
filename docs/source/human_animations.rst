Human Animations
=================

We currently do not have a reactive human simulation but use a playback animation system.
There are a couple of human animations converted from the CMU motion capture data set available already.
In addition to that, we are going to record task-specific animations in our lab.

The available human animation names are listed in ``human_robot_gym/models/assets/human/animations``.
In the future, this will be moved to a separated repository.

Each animation can be adapted (e.g., starting position and orientation) in ``human_robot_gym/models/assets/human/animations/animation_info.json``.

The ``HumanEnv`` currently supports the following options for the human animations:

    - ``human_animation_names (list[str])``: Human animations to play (must match the names in ``human_robot_gym/models/assets/human/animations``)
    - ``base_human_pos_offset (list[double])``: Base human animation offset (offset for **all** animations)
    - ``human_animation_freq (double)``: Speed of the human animation in FPS (must be adapted to the animation type CMU, custom recording).
    - ``human_rand (list[double])``: Max. randomization of the human [x-pos, y-pos, z-angle (rad)]

A detailed description of how to record new data can be found `here (access required) <https://gitlab.lrz.de/cps-robotics/modular-robot-toolbox/-/wikis/Real%20Robot%20Setup/5.%20Recording%20Vicon%20Data>`_. 