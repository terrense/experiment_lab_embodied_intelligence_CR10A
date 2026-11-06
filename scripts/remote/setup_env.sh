#!/usr/bin/env bash
# Create the python environment on the remote compute node (run ON THE SERVER).
#
#   bash scripts/remote/setup_env.sh
#
# The environment is a plain venv - no ROS. That is possible because the learning code only
# needs:
#   * the expanded URDF (data/cache/ex_cr10_calibrated_1.urdf, shipped with the repository) and
#     the mesh files (ros2_ws/src/ex_description/meshes);
#   * a python with mujoco, numpy, scipy, gymnasium, stable-baselines3 and torch.
# The xacro expansion happens locally and is cached, so `xacro` is NOT needed here.
#
# The GPU is not used by the M5 training (the MuJoCo plant is CPU-bound and the policy is a
# small MLP); the value of the server is the number of CPU cores, i.e. how many seeds can be
# trained in parallel. setup_env.sh installs the CUDA build of torch anyway, because the
# planned residual-RL arm (B7) does use it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

PY="${PYTHON:-python3}"
VENV="${VENV:-$ROOT/../ex_rlenv_remote}"

echo "== python: $($PY --version 2>&1)   cores: $(nproc)   memory: $(free -g | awk '/Mem:/{print $2}') GB"
echo "== creating the venv at $VENV"
$PY -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"

python -m pip install --upgrade pip wheel
echo "== installing the dependencies"
python -m pip install \
  "numpy<2" "scipy" "matplotlib" "pyyaml" "mujoco>=3.1,<3.4" \
  "gymnasium" "stable-baselines3>=2.4" "torch" "pandas"

# the CUDA build of torch, if the node has an NVIDIA driver. Failure is not fatal: the M5
# training does not need it, and the residual-RL arm can run on the CPU.
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "== an NVIDIA GPU is present:"
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv || true
  echo "   installing the CUDA build of torch"
  python -m pip install torch --index-url https://download.pytorch.org/whl/cu121 || \
    echo "   (the CUDA wheel install failed; continuing with the CPU build)"
else
  echo "== no NVIDIA GPU on this node"
fi

echo "== verifying the imports and the plant"
cd "$ROOT/learning"
python - <<'PY'
import numpy, scipy, mujoco, gymnasium, stable_baselines3, torch
print("numpy", numpy.__version__, "| scipy", scipy.__version__, "| mujoco", mujoco.__version__)
print("gymnasium", gymnasium.__version__, "| sb3", stable_baselines3.__version__)
print("torch", torch.__version__, "| cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("   device:", torch.cuda.get_device_name(0))
import sys
sys.path.insert(0, ".")
from ex_learning.envs.cr10a_reach import expanded_urdf_text, make_model
urdf = expanded_urdf_text(True)
m = make_model()
print("URDF", len(urdf), "chars | model built:", m.nq, "DoF,", m.nbody, "bodies")
PY

echo
echo "== recommended parallelism for seed training"
echo "   REMOTE_JOBS=$(( $(nproc) / (12 + 1) ))   (12 envs + 1 training process per job)"
echo "   a 5M-timestep run took 37-41 min on the laptop; the server's cores, not its GPU,"
echo "   are what shortens it."
