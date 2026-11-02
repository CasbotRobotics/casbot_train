#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认参数设置
DEFAULT_ARCH="amd64"
DEFAULT_PACKAGE_NAME=""
DEFAULT_PACKAGE_VERSION=""

# 初始化变量
TARGET_ARCH="${DEFAULT_ARCH}"
PACKAGE_NAME="${DEFAULT_PACKAGE_NAME}"
PACKAGE_VERSION="${DEFAULT_PACKAGE_VERSION}"

# 解析参数：
# 第1个参数：可选，架构（amd64/arm64），默认amd64
# 第2个参数：可选，包名，默认casbot-motion
# 第3个参数：可选，版本，默认0.1.0
if [ $# -ge 1 ] && [ "$1" = "amd64" -o "$1" = "arm64" ]; then
    TARGET_ARCH="$1"
    # 若提供了第2个参数，更新包名
    if [ $# -ge 2 ]; then
        PACKAGE_NAME="$2"
        # 若提供了第3个参数，更新版本
        if [ $# -ge 3 ]; then
            PACKAGE_VERSION="$3"
        fi
    fi
else
    # 第1个参数不是包名（非架构），则按包名、版本解析
    if [ $# -ge 1 ]; then
        PACKAGE_NAME="$1"
        if [ $# -ge 2 ]; then
            PACKAGE_VERSION="$2"
        fi
    fi
fi

# 根据架构选择镜像（核心修正：严格按解析后的ARCH赋值）
if [ "${TARGET_ARCH}" = "arm64" ]; then
    DOCKER_IMAGE="hub.casbot.dev/environment/casbot-arm64-builder:latest"
else
    DOCKER_IMAGE="hub.casbot.dev/environment/casbot-amd64-builder:latest"
fi

# Prefer the official crb_ros_msg package when it contains JointStateData.
# Otherwise make.sh builds the exact interface from the compatibility source
# carried by this project.
function has_joint_state_data() {
    local prefix="$1"
    [ -f "${prefix}/include/crb_ros_msg/crb_ros_msg/msg/joint_state_data.hpp" ] || \
        [ -f "${prefix}/include/crb_ros_msg/msg/joint_state_data.hpp" ]
}

if [ "${TARGET_ARCH}" = "arm64" ]; then
    DEFAULT_HL_MOTION="${SCRIPT_DIR}/build/crb_ros_msg_arm64"
    for candidate in \
        "${SCRIPT_DIR}/build/crb_ros_msg_aarch64" \
        "/home/casbot/rl_controller/build/crb_ros_msg_aarch64" \
        "/home/casbot/HLmotion/install_aarch64" \
        "/home/casbot/casbot_infer/casbot_amp/HLmotion/install_aarch64"; do
        if has_joint_state_data "${candidate}"; then
            DEFAULT_HL_MOTION="${candidate}"
            break
        fi
    done
    if [ -n "${HL_MOTION_PREFIX:-}" ] && ! has_joint_state_data "${HL_MOTION_PREFIX}"; then
        echo "提示：${HL_MOTION_PREFIX} 不含 JointStateData，将在容器内构建兼容接口"
    fi
else
    DEFAULT_HL_MOTION="/workspace/hl_motion"
fi
HL_MOTION_PREFIX="${HL_MOTION_PREFIX:-${DEFAULT_HL_MOTION}}"

# 构建命令
echo "开始构建："
echo "架构: ${TARGET_ARCH}"
echo "镜像: ${DOCKER_IMAGE}"
echo "HLMotion: ${HL_MOTION_PREFIX}"

DOCKER_ARGS=(
    --rm
    -v "${SCRIPT_DIR}/:${SCRIPT_DIR}/"
    -e "HL_MOTION_PREFIX=${HL_MOTION_PREFIX}"
    -e "BUILD_ENV=${TARGET_ARCH}"
    -w "${SCRIPT_DIR}"
)

case "${HL_MOTION_PREFIX}/" in
    "${SCRIPT_DIR}/"*)
        ;;
    *)
        if [ -d "${HL_MOTION_PREFIX}" ]; then
            DOCKER_ARGS+=(
                -v "${HL_MOTION_PREFIX}:${HL_MOTION_PREFIX}:ro"
            )
        fi
        ;;
esac

if [[ -z ${PACKAGE_NAME} && -z ${PACKAGE_VERSION} ]]; then
    echo ">>>>>>>>>"
    docker run "${DOCKER_ARGS[@]}" \
        "${DOCKER_IMAGE}" \
        ./make.sh
else
    echo "包名: ${PACKAGE_NAME}"
    echo "版本: ${PACKAGE_VERSION}"
    echo ">>>>>>>>>"
    docker run "${DOCKER_ARGS[@]}" \
        "${DOCKER_IMAGE}" \
        ./make.sh "${PACKAGE_NAME}" "${PACKAGE_VERSION}" ${TARGET_ARCH}
fi

echo "构建完成"
