#!/usr/bin/env bash
# One-shot setup of this workspace on Ubuntu 22.04 (native, WSL2, or the macOS UTM VM).
#
#   cd ~/ugv_rover && bash scripts/setup_ubuntu.sh
#
# Installs ROS 2 Humble, the packages this repo needs, Webots support (webots_ros2),
# pulls the Git LFS model files, runs rosdep, builds the workspace and adds the
# environment to ~/.bashrc. Safe to run again: every step checks what is already done.
#
# Options:
#   --no-build     install everything but do not run colcon build
#   --no-upgrade   skip "apt-get upgrade"
#   --no-lfs       do not download the Git LFS model files (CI does not need them to build)

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASHRC="$HOME/.bashrc"
DO_BUILD=1
DO_UPGRADE=1
DO_LFS=1
for arg in "$@"; do
  case "$arg" in
    --no-build) DO_BUILD=0 ;;
    --no-upgrade) DO_UPGRADE=0 ;;
    --no-lfs) DO_LFS=0 ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARNING: %s\033[0m\n' "$*"; }
add_line() { grep -qxF "$1" "$BASHRC" 2>/dev/null || echo "$1" >> "$BASHRC"; }
is_wsl() { grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; }

# ---------------------------------------------------------------------------
step "Checking the system"
. /etc/os-release
if [ "${VERSION_ID:-}" != "22.04" ]; then
  echo "This workspace needs Ubuntu 22.04 (ROS 2 Humble). Found: ${PRETTY_NAME:-unknown}." >&2
  echo "On Windows: wsl --install -d Ubuntu-22.04 (see docs/setup/WINDOWS_WSL2.md)." >&2
  exit 1
fi
case "$WS" in
  /mnt/*) warn "The workspace is on the Windows drive ($WS). Clone it into ~ instead (much faster builds)." ;;
esac
echo "Workspace: $WS"
echo "Ubuntu:    $PRETTY_NAME ($(dpkg --print-architecture))"
is_wsl && echo "Running in WSL"

# ---------------------------------------------------------------------------
step "Locale and base tools"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -qq
sudo apt-get install -y -qq locales curl software-properties-common gnupg lsb-release git >/dev/null
if ! locale -a | grep -qi '^en_US.utf8$'; then
  sudo locale-gen en_US en_US.UTF-8 >/dev/null
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
fi
sudo add-apt-repository -y universe >/dev/null

# ---------------------------------------------------------------------------
step "ROS 2 apt repository (ros2-apt-source)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  # Follow the releases/latest redirect instead of the GitHub API (rate limited per IP).
  version=$(curl -fsSLI -o /dev/null -w '%{url_effective}' \
            https://github.com/ros-infrastructure/ros-apt-source/releases/latest)
  version=${version##*/}
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb >/dev/null
  rm -f /tmp/ros2-apt-source.deb
  echo "Installed ros2-apt-source ${version}"
else
  echo "Already configured"
fi
sudo apt-get update -qq
if [ "$DO_UPGRADE" = 1 ]; then
  sudo apt-get -y -qq upgrade >/dev/null
fi

# ---------------------------------------------------------------------------
step "Installing ROS 2 Humble and the packages this repo uses (this takes a while)"
# Same list as build_first.sh, plus the Webots simulator support.
# The log is written as the user on purpose (SC2024).
# shellcheck disable=SC2024
sudo apt-get install -y -q \
  gh git-lfs python3-pip python3-rosdep python3-vcstool python3-colcon-common-extensions \
  python3-colcon-argcomplete ros-dev-tools \
  ros-humble-desktop ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization ros-humble-slam-toolbox \
  ros-humble-cartographer-ros ros-humble-cartographer-ros-msgs ros-humble-cartographer-rviz ros-humble-libg2o \
  ros-humble-rtabmap-ros ros-humble-rosbridge-suite ros-humble-rqt ros-humble-rqt-common-plugins \
  ros-humble-v4l2-camera ros-humble-usb-cam ros-humble-image-transport-plugins ros-humble-tf2-tools \
  ros-humble-depthai ros-humble-depthai-ros ros-humble-depthai-bridge ros-humble-depthai-ros-driver \
  ros-humble-depthai-ros-msgs ros-humble-depthai-descriptions ros-humble-depthai-examples ros-humble-depthai-filters \
  ros-humble-webots-ros2 ros-humble-depth-image-proc ros-humble-teleop-twist-keyboard \
  libsuitesparse-dev alsa-utils espeak-ng \
  > /tmp/ugv_setup_apt.log 2>&1 || { tail -20 /tmp/ugv_setup_apt.log; echo "apt-get install failed, full log: /tmp/ugv_setup_apt.log" >&2; exit 1; }
echo "ROS 2 $(dpkg-query -W -f='${Version}' ros-humble-desktop) and webots_ros2 $(dpkg-query -W -f='${Version}' ros-humble-webots-ros2) installed"

# ---------------------------------------------------------------------------
step "Git LFS model files"
if [ "$DO_LFS" = 1 ]; then
  git lfs install >/dev/null
  ( cd "$WS" && git lfs pull )
  pointers=0
  while IFS= read -r f; do
    if [ -f "$WS/$f" ] && head -c 200 "$WS/$f" | grep -q 'git-lfs.github.com/spec'; then
      echo "  still a pointer: $f"; pointers=$((pointers + 1))
    fi
  done < <(cd "$WS" && git lfs ls-files -n)
  if [ "$pointers" = 0 ]; then
    echo "All LFS files downloaded"
  else
    warn "$pointers LFS files are still pointers (see docs/TROUBLESHOOTING.md)"
  fi
else
  echo "Skipped (--no-lfs)"
fi

# ---------------------------------------------------------------------------
step "rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro humble >/dev/null
set +u
source /opt/ros/humble/setup.bash
set -u
# -r: continue past the known harmless 'cmake_modules' key of rf2o_laser_odometry
( cd "$WS" && rosdep install --from-paths src --ignore-src -y -r --rosdistro humble 2>&1 \
    | grep -vE '^(Get|Selecting|Preparing|Unpacking|Setting up|Processing|Reading|Building)' | tail -5 ) || true
echo "(The 'cmake_modules' message from rf2o_laser_odometry is expected and harmless.)"

# ---------------------------------------------------------------------------
step "Environment in ~/.bashrc"
add_line "source /opt/ros/humble/setup.bash"
add_line "[ -f \"$WS/install/setup.bash\" ] && source \"$WS/install/setup.bash\""
add_line "export UGV_MODEL=ugv_rover"
add_line "export LDLIDAR_MODEL=ld19"
if is_wsl; then
  add_line 'export WEBOTS_HOME="/mnt/c/Program Files/Webots"'
fi
echo "Done (open a new terminal to pick it up)"

# ---------------------------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
  step "Building the workspace"
  set +u
  cd "$WS"
  export UGV_MODEL=ugv_rover LDLIDAR_MODEL=ld19
  # gz_ros2_control is only used by the Gazebo Harmonic setup; skipping it saves build time.
  if colcon build --symlink-install --packages-skip gz_ros2_control > /tmp/ugv_setup_build.log 2>&1; then
    grep -E 'Summary' /tmp/ugv_setup_build.log
  else
    grep -E 'Failed|Summary' /tmp/ugv_setup_build.log
    echo "colcon build failed, full log: /tmp/ugv_setup_build.log" >&2
    exit 1
  fi
  set -u
fi

# ---------------------------------------------------------------------------
step "Webots"
if is_wsl; then
  if [ -x "/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" ]; then
    echo "Found $(cat '/mnt/c/Program Files/Webots/resources/version.txt' 2>/dev/null) in C:\\Program Files\\Webots"
  else
    warn "Webots is not installed on Windows. Install R2025a for all users from"
    echo "  https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe"
  fi
elif [ -n "${WEBOTS_SHARED_FOLDER:-}" ]; then
  echo "macOS VM mode: WEBOTS_SHARED_FOLDER=$WEBOTS_SHARED_FOLDER (see docs/setup/MACOS.md)"
elif ! command -v webots >/dev/null 2>&1 && [ ! -x /usr/local/webots/webots ]; then
  warn "Webots is not installed. On native Ubuntu: see docs/setup/LINUX.md (Webots R2025a .deb)."
fi

step "All done"
echo "Open a new terminal, then try:"
echo "  ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true"
