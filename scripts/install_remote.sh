#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOCAL_DIR="${SCRIPT_DIR}/../build/install_arm64/"
REMOTE_HOST="${REMOTE_HOST:-172.16.0.10}"
REMOTE_USER="${REMOTE_USER:-nvidia}"
REMOTE_PASS="${REMOTE_PASS:-}"
SUDO_PASS="${SUDO_PASS:-}"
REMOTE_DIR="/workspace/rl_controller/"
SSH_PORT="${SSH_PORT:-22}"

check_sshpass() {
    if ! command -v sshpass &> /dev/null; then
        echo "❌ 错误：未检测到 sshpass, 请先安装！"
        echo
        echo "  sudo apt update && sudo apt install -y sshpass"
        echo
        exit 1
    fi
}

check_rsync() {
    if ! command -v rsync &> /dev/null; then
        echo "❌ 错误：未检测到 rsync, 请先安装！"
        echo
        echo "  sudo apt update && sudo apt install -y rsync"
        echo
        exit 1
    fi
}

check_remote_connectivity() {
    echo "🔍 开始检测远程服务器连通性：${REMOTE_HOST}:${SSH_PORT}"
    
    # 1. 先 ping 检测网络可达性（超时5秒）
    if ! ping -c 1 -W 5 "${REMOTE_HOST}" &> /dev/null; then
        echo "❌ 网络不通：无法 ping 通 ${REMOTE_HOST}"
        exit 1
    fi
    echo "✅ 网络可达: ping ${REMOTE_HOST} 成功"

    # 2. 检测 SSH 端口是否开放（超时10秒）
    if ! timeout 10 bash -c "echo > /dev/tcp/${REMOTE_HOST}/${SSH_PORT}" &> /dev/null; then
        echo "❌ SSH 端口不通：${REMOTE_HOST}:${SSH_PORT} 未开放"
        exit 1
    fi
    echo "✅ SSH 端口开放：${REMOTE_HOST}:${SSH_PORT} 可访问"

    # 3. 检测 SSH 账号密码是否正确（超时10秒）
    if ! timeout 10 sshpass -p "${REMOTE_PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${REMOTE_USER}@${REMOTE_HOST}" "echo 'SSH_AUTH_OK'" &> /dev/null; then
        echo "❌ SSH 认证失败：账号/密码错误或无登录权限: ${REMOTE_USER}"
        exit 1
    fi
    echo "✅ SSH 认证成功：${REMOTE_USER}@${REMOTE_HOST} 可登录"
}

create_remote_dir() {
    if sshpass -p "${REMOTE_PASS}" ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" "echo ${SUDO_PASS} | sudo -S mkdir -p ${REMOTE_DIR}"; then
        # 创建后修改目录归属为 nvidia 用户（避免权限问题）
        sshpass -p "${REMOTE_PASS}" ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" "echo ${SUDO_PASS} | sudo -S chown -R ${REMOTE_USER}:${REMOTE_USER} ${REMOTE_DIR}"
        return 0
    fi
    return 1
}

main() {
    # 检查本地目录
    echo "LOCAL_DIR: ${LOCAL_DIR}"
    if [ ! -d "${LOCAL_DIR}" ]; then
        echo "❌ 错误：本地目录 ${LOCAL_DIR} 不存在！"
        exit 1
    fi

    check_sshpass

    check_rsync

    if [ -z "${REMOTE_PASS}" ]; then
        read -r -s -p "请输入 ${REMOTE_USER}@${REMOTE_HOST} 的 SSH 密码: " REMOTE_PASS
        echo
    fi
    SUDO_PASS="${SUDO_PASS:-${REMOTE_PASS}}"

    check_remote_connectivity
    

    # 创建远程目录（如果不存在）
    echo "🔍 检查并创建远程目录：${REMOTE_HOST}:${REMOTE_DIR}"
    create_remote_dir

    # 执行 rsync（通过 sshpass 传递密码）
    echo "🚀 开始同步文件..."
    echo "target ip: ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}"
    sshpass -p "${REMOTE_PASS}" rsync -av --delete -e "ssh -o StrictHostKeyChecking=no" "${LOCAL_DIR}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}"

    # 检查执行结果
    if [ $? -eq 0 ]; then
        echo "✅ 同步完成：本地 ${LOCAL_DIR} → 远程 ${REMOTE_HOST}:${REMOTE_DIR}"
    else
        echo "❌ 同步失败！"
        exit 1
    fi

    echo "📊 验证远程目录状态："
    sshpass -p "${REMOTE_PASS}" ssh -o StrictHostKeyChecking=no "${REMOTE_USER}@${REMOTE_HOST}" "
        echo '远程目录大小：'
        du -sh ${REMOTE_DIR}
    " || true
}


main
