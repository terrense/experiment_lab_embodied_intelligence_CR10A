#!/usr/bin/env bash
# Install MuJoCo (Python) into the EmbodiedX-Humble distro WITHOUT touching the system numpy
# (ROS Humble is built against numpy 1.21; a numpy 2 in ~/.local would break rosidl messages).
#   1. on Windows (has internet via proxy):
#      python -m pip download --only-binary=:all: --platform manylinux2014_x86_64 --platform manylinux_2_28_x86_64 \
#             --python-version 3.10 --implementation cp -d <dir> "mujoco>=3.1,<3.4" "pip==24.2"
#   2. in WSL:  bash scripts/setup_mujoco.sh <dir>
# WSL has no pip; pip is run straight from its wheel. numpy is NOT installed (system 1.21.5 is used;
# mujoco declares an unversioned numpy requirement and works with it - verified by unit tests).
set -euo pipefail
W="${1:?wheel directory}"
PIP="python3 $(ls "$W"/pip-*.whl | head -1)/pip"
$PIP install --user --no-index --find-links="$W" mujoco
python3 -c "import numpy, mujoco; print('numpy', numpy.__version__, 'mujoco', mujoco.__version__)"
