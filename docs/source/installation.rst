Installation
============



Clone the repo with submodules
------------------------------

.. code-block:: bash

    git clone --recurse-submodules git@gitlab.lrz.de:cps-rl/human-robot-gym.git

Install MuJoCo
--------------
Steps:

1. Download the MuJoCo version 2.1 binaries for `Linux <https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz>`_ or `OSX <https://mujoco.org/download/mujoco210-macos-x86_64.tar.gz>`_. 
2. Extract the downloaded ``mujoco210`` directory into ``~/.mujoco/mujoco210``.

If you want to specify a nonstandard location for the package,
use the env variable ``MUJOCO_PY_MUJOCO_PATH``.

Under linux, make sure to install: 

.. code-block:: bash
    
    sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3 libgtest-dev


Setup anaconda environment
--------------------------
If you haven't done already, `install anaconda <https://docs.anaconda.com/anaconda/install/linux/>`_.
Add ``conda-forge`` to your channels with

.. code-block:: bash

    conda config --add channels conda-forge
    conda config --set channel_priority strict

and create the ``hrgym`` conda environment:

.. code-block:: bash
    
    conda env create -f environment.yml
    conda activate hrgym

All requirements will automatically get installed by conda.

Install the failsafe controller / safety shield
-----------------------------------------------
The installation requires ``gcc``, ``c++>=17``, and ``Eigen3`` version 3.4 (`download it here <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_.
Set the path to your eigen3 installation to this env variable, e.g.,

.. code-block:: bash

    export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"

Now run

.. code-block:: bash

    cd human-robot-gym/human_robot_gym/controllers/failsafe_controller/sara-shield
    pip install -r requirements.txt
    python setup.py install

Install the human-robot-gym
---------------------------

.. code-block:: bash

    cd human-robot-gym
    pip install -e .

Add to your ``~/.bashrc`` 
-------------------------
E.g. with ``nano ~/.bashrc``:

.. code-block:: bash
    
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/thummj/.mujoco/mujoco210/bin
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
