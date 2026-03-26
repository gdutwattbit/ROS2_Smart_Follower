#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WITH_GPIO=1
WITH_ONNXRUNTIME=1
INSTALL_ROS=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

log() {
  echo -e "\033[1;32m[INFO]\033[0m $*"
}

warn() {
  echo -e "\033[1;33m[WARN]\033[0m $*" >&2
}

die() {
  echo -e "\033[1;31m[ERROR]\033[0m $*" >&2
  exit 1
}

show_help() {
  cat <<'EOF'
用法: bash scripts/install_dependencies.sh [选项]

选项:
  --without-gpio         跳过 libgpiod 安装
  --without-onnxruntime  跳过 ONNX Runtime 安装尝试
  --install-ros          若系统未安装 ROS，则尝试通过 apt 安装 ros-$ROS_DISTRO-desktop
  -h, --help             显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --without-gpio)
      WITH_GPIO=0
      shift
      ;;
    --without-onnxruntime)
      WITH_ONNXRUNTIME=0
      shift
      ;;
    --install-ros)
      INSTALL_ROS=1
      shift
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      die "未知参数: $1"
      ;;
  esac
done

command -v apt-get >/dev/null 2>&1 || die "当前系统不支持 apt-get，本脚本目标平台是 Ubuntu 22.04"
command -v python3 >/dev/null 2>&1 || die "未找到 python3"

install_if_available() {
  local pkg="$1"
  if apt-cache show "${pkg}" >/dev/null 2>&1; then
    log "安装可选系统包: ${pkg}"
    "${SUDO[@]}" apt-get install -y "${pkg}"
    return 0
  fi
  return 1
}

log "进入仓库根目录: ${REPO_ROOT}"
cd "${REPO_ROOT}"

BASE_PACKAGES=(
  build-essential
  cmake
  git
  curl
  wget
  unzip
  pkg-config
  python3-pip
  python3-rosdep
  python3-colcon-common-extensions
  python3-vcstool
  libopencv-dev
  libeigen3-dev
)

if [[ "${WITH_GPIO}" -eq 1 ]]; then
  BASE_PACKAGES+=(
    libgpiod2
    libgpiod-dev
    gpiod
  )
fi

log "更新 apt 索引"
"${SUDO[@]}" apt-get update

log "安装基础依赖"
"${SUDO[@]}" apt-get install -y "${BASE_PACKAGES[@]}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  if [[ "${INSTALL_ROS}" -eq 1 ]]; then
    if apt-cache show "ros-${ROS_DISTRO}-desktop" >/dev/null 2>&1; then
      log "开始安装 ros-${ROS_DISTRO}-desktop"
      "${SUDO[@]}" apt-get install -y "ros-${ROS_DISTRO}-desktop"
    else
      die "未检测到 ros-${ROS_DISTRO}-desktop，请先配置 ROS 2 apt 源"
    fi
  else
    die "未检测到 /opt/ros/${ROS_DISTRO}/setup.bash，请先安装 ROS 2 ${ROS_DISTRO}"
  fi
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if ! dpkg -s python3-ament-index-python >/dev/null 2>&1; then
  install_if_available python3-ament-index-python || \
    install_if_available "ros-${ROS_DISTRO}-ament-index-python" || \
    warn "未找到 ament_index_python 对应系统包，请在 ROS 环境中确认该模块可用"
fi

if [[ "${WITH_ONNXRUNTIME}" -eq 1 ]]; then
  if ! dpkg -s libonnxruntime-dev >/dev/null 2>&1; then
    if ! install_if_available libonnxruntime-dev; then
      warn "系统仓库未提供 libonnxruntime-dev，感知节点仍可编译，但会退化为 stub inference"
      warn "如需真实推理，请自行安装 ONNX Runtime 并设置 ONNXRUNTIME_ROOT，或放到 third_party/onnxruntime*"
    fi
  fi
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "初始化 rosdep"
  "${SUDO[@]}" rosdep init
fi

log "更新 rosdep 数据库"
rosdep update

log "通过 rosdep 安装工作区依赖（跳过 turn_on_wheeltec_robot）"
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  -r -y \
  --skip-keys="turn_on_wheeltec_robot"

mkdir -p models

if [[ ! -f models/yolo26n_static_256x320_simplify_e2e.onnx ]]; then
  warn "未找到 models/yolo26n_static_256x320_simplify_e2e.onnx"
fi

if [[ ! -f models/osnet_x0_5_512.onnx ]]; then
  warn "未找到 models/osnet_x0_5_512.onnx"
fi

warn "若未安装 turn_on_wheeltec_robot，可使用 smart_follower.launch.py bringup_robot:=false 仅启动本项目链路"

log "依赖安装完成"
log "下一步可执行:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch smart_follower_bringup smart_follower.launch.py robot_ns:=robot1 bringup_robot:=false bringup_camera:=false"
