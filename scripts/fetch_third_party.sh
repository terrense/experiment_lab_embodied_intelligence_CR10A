#!/usr/bin/env bash
# Fetch pinned third-party sources (tarballs by commit; much faster than git
# over slow links). Each ref goes to its own directory; nothing is executed.
set -euo pipefail
cd "$(dirname "$0")/../third_party"
fetch() {  # fetch <dir> <commit>
  local dir=$1 sha=$2
  [ -d "$dir" ] && { echo "$dir exists, skip"; return; }
  mkdir -p "$dir.tmp"
  curl -fsSL --retry 3 ${HTTPS_PROXY:+-x $HTTPS_PROXY} \
    "https://codeload.github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tar.gz/$sha" | tar xz -C "$dir.tmp" --strip-components=1
  mv "$dir.tmp" "$dir"; echo "$dir <- $sha"
}
fetch dobot_main  ec201c40d5db63185a3593ec9c39f80fe25aa527
fetch dobot_v4opt 0f21e9839888f188ac06b0a9fea622d13a29a74f
