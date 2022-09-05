#!/bin/bash --login
# The --login ensures the bash configuration is loaded,
# enabling Conda.

# Enable strict mode.
set -euo pipefail
# ... Run whatever commands ...

# Temporarily disable strict mode and activate conda:
set +euo pipefail
conda activate hrgym

# Re-enable strict mode:
set -euo pipefail

# exec the final command:
cd /tmp/human-robot-gym/human_robot_gym/controllers/failsafe_controller/sara-shield && sudo rm -rf build safety_shield/build && sudo -E $CONDA_PREFIX/bin/python setup.py install
cd /tmp/human-robot-gym/ && pip install -e .

exec "$@"