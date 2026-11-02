#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


SOURCE_DIR="${SCRIPT_DIR}/../build/install_amd64"
TARGET_DIR="/workspace/rl_controller"

SUDO='sudo'

if [ "$(id -u)" -eq 0 ]; then
    SUDO=''
fi

# 1. 检测源目录是否不为空
if [ -z "$(ls -A "${SOURCE_DIR}" 2>/dev/null)" ]; then
    echo "错误：源目录 ${SOURCE_DIR} 为空或不存在，退出脚本"
    exit 1
fi

# 2. 处理目标目录
if [ -d "${TARGET_DIR}" ]; then
    echo "删除已存在的目标目录：${TARGET_DIR}"
    ${SUDO} rm -rf "${TARGET_DIR}"
fi

# 3. 创建目标目录
echo "创建目标目录：${TARGET_DIR}"
${SUDO} mkdir -p "${TARGET_DIR}"

# 4. 拷贝源目录下所有内容到目标目录
echo "正在将 ${SOURCE_DIR} 下的内容拷贝到 ${TARGET_DIR}..."
${SUDO} cp -r "${SOURCE_DIR}"/* "${TARGET_DIR}/"
# sudo cp may preserve restrictive directories from a container build. Ensure
# the account launching the controller can read models/config and execute it.
${SUDO} chmod -R a+rX "${TARGET_DIR}"
${SUDO} find "${TARGET_DIR}" -type f -name 'rl_controller_runner' -exec chmod a+x {} \;


# 5. 检查并执行 setup.sh
if [ ! -f "${TARGET_DIR}/setup.sh" ]; then
    echo "错误：${TARGET_DIR}/setup.sh 不存在，无法初始化环境"
    exit 1
fi
echo "执行环境初始化脚本..."
if [ ! -f "/workspace/hl_motion/setup.bash" ]; then
    echo "错误：/workspace/hl_motion/setup.bash 不存在，无法加载 crb_ros_msg 运行库" >&2
    exit 1
fi
source /workspace/hl_motion/setup.bash
source "${TARGET_DIR}/setup.sh"

export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# 6. 确保可执行文件权限并运行
${SUDO} chmod +x "${TARGET_DIR}/bin/rl_controller_runner"  # 可选，根据文件实际权限调整


echo "更新动态链接库缓存..."
${SUDO} ldconfig

echo "启动 rl_controller 程序..."
"${TARGET_DIR}/bin/rl_controller_runner"
