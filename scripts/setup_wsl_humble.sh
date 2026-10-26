#!/usr/bin/env bash
# Provision a DEDICATED WSL2 distro for EMBODIED-X with ROS 2 Humble.
#
# Run as root INSIDE the dedicated distro only (never in a distro used by other
# projects):
#   wsl --install Ubuntu-22.04 --name EmbodiedX-Humble --location C:\WSL\EmbodiedX-Humble --no-launch --web-download
#   wsl -d EmbodiedX-Humble -u root -- bash /mnt/c/Users/<you>/embodied-x/scripts/setup_wsl_humble.sh
#
# Changes made (all inside this distro):
#   * user 'ex' with passwordless sudo, set as default WSL user
#   * apt sources -> TUNA mirror (Ubuntu + ROS 2)   [MIRROR=default to skip]
#   * packages: ros-humble-desktop, xacro, rosbag2, colcon, pytest, numpy
set -euo pipefail

EX_USER="${EX_USER:-ex}"
MIRROR="${MIRROR:-tuna}"
export DEBIAN_FRONTEND=noninteractive

[[ $(id -u) -eq 0 ]] || { echo "run as root"; exit 1; }
grep -q 'VERSION_ID="22.04"' /etc/os-release || { echo "Ubuntu 22.04 required for Humble"; exit 1; }

# ---- user ------------------------------------------------------------------
if ! id "$EX_USER" &>/dev/null; then
  useradd -m -s /bin/bash -G sudo "$EX_USER"
  echo "$EX_USER ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/90-$EX_USER"
  chmod 440 "/etc/sudoers.d/90-$EX_USER"
fi
if ! grep -q "^default=$EX_USER" /etc/wsl.conf 2>/dev/null; then
  printf '[user]\ndefault=%s\n' "$EX_USER" >> /etc/wsl.conf
fi

# ---- apt sources -----------------------------------------------------------
if [[ "$MIRROR" == "tuna" ]]; then
  UBU=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
  ROS=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
  sed -i -E "s#https?://(archive|security)\.ubuntu\.com/ubuntu/?#${UBU}/#g" /etc/apt/sources.list
else
  ROS=http://packages.ros.org/ros2/ubuntu
fi
apt-get update
apt-get install -y curl gnupg lsb-release ca-certificates locales software-properties-common
locale-gen en_US.UTF-8 >/dev/null
# ROS key: shipped by ros/rosdistro; the mirror serves the same signed packages.
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL --retry 5 https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    || curl -fsSL --retry 5 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS} jammy main" \
  > /etc/apt/sources.list.d/ros2.list

# ---- packages --------------------------------------------------------------
apt-get update
apt-get install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rosbag2-storage-default-plugins \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest python3-numpy python3-yaml \
  git

# ---- shell env for the dev user -------------------------------------------
BASHRC="/home/$EX_USER/.bashrc"
grep -q 'opt/ros/humble/setup.bash' "$BASHRC" || cat >> "$BASHRC" <<'EOF'

# --- EMBODIED-X ---
source /opt/ros/humble/setup.bash
export EX_ROOT=/mnt/c/Users/HGDOCTOR/embodied-x
export EX_WS_OUT=$HOME/ex_ws_out
[ -f $EX_WS_OUT/install/setup.bash ] && source $EX_WS_OUT/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
EOF

echo "OK: ROS 2 $(ls /opt/ros) installed for user $EX_USER. Restart the distro: wsl --terminate EmbodiedX-Humble"
