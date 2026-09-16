#!/usr/bin/env bash
# Source the colcon overlay when it exists so `pixi run` can find built packages.
_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "${_root}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_root}/install/setup.bash"
fi
# Prepend so sitecustomize.py is imported before python_qt_binding.
export PYTHONPATH="${_root}/scripts/pythonpath${PYTHONPATH:+:$PYTHONPATH}"
# macOS (and VPNs) break multicast DDS; keep ROS 2 discovery on loopback.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${_root}/scripts/cyclonedds.xml}"
unset _root
