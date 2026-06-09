#!/usr/bin/env bash
# Runs INSIDE the ros:jazzy packaging container. Builds pairs_uav_hw_api and its
# PAIRS deps in order: pairs_msgs -> pairs_lib -> pairs_uav_hw_api.
set -eo pipefail
ROS_DISTRO_NAME="jazzy"; OS_NAME="ubuntu"; OS_VERSION="noble"
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"; set -u
LOCAL_RULES="/src/pairs_uav_hw_api/packaging/rosdep/pairs.yaml"
echo "yaml file://${LOCAL_RULES}" | sudo tee /etc/ros/rosdep/sources.list.d/10-pairs.list >/dev/null
rosdep update
sudo apt-get update
mkdir -p /output
build_one() {
  local pkg_dir="$1"; local pkg_name; pkg_name="$(basename "$pkg_dir")"
  [ -d "$pkg_dir" ] || { echo "ERROR: '$pkg_name' not at $pkg_dir (checked out on ros2?)" >&2; exit 1; }
  echo "=== Building $pkg_name ==="
  local work="/ws/${pkg_name}"; rm -rf "$work"; cp -a "$pkg_dir" "$work"; cd "$work"; rm -rf debian obj-*
  rosdep install --from-paths . --ignore-src -r -y || true
  bloom-generate rosdebian --os-name "$OS_NAME" --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO_NAME"
  fakeroot debian/rules binary
  for deb in /ws/ros-${ROS_DISTRO_NAME}-*"${pkg_name//_/-}"*.deb ../ros-${ROS_DISTRO_NAME}-*.deb; do
    [ -f "$deb" ] || continue; cp -v "$deb" /output/; apt-get install -y "$deb" || dpkg -i "$deb" || true
  done
}
build_one /src/pairs_msgs
build_one /src/pairs_lib
build_one /src/pairs_uav_hw_api
echo "=== Done ==="; ls -1 /output/*.deb
