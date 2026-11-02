#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_DIR="${SCRIPT_DIR}"

BUILD_DIR="${PROJECT_DIR}/build"
PROJECT_BUILD_DIR="${BUILD_DIR}"

SOFTWARE_INSTALL_PATH="/workspace/rl_controller"

SOFTWARE_ARCH=""

NUM_CORE=8

ENABLE_PACKAGE="false"

G_GITBRANCH=""
G_GITCOMMITID=""

function get_build_core_num() {
    # 获取本地CPU核心数
    local total_cores=$(nproc)
    local build_cores

    if [ "${total_cores}" -le 8 ]; then
        # 核心数≤8时，使用核心数减1
        build_cores=$((total_cores - 1))
    elif [ "${total_cores}" -le 16 ]; then
        #  核心数>8且≤16时，固定使用8
        build_cores=8
    else
        # 核心数>16时，使用核心数的1/2（向下取整）
        build_cores=$((total_cores / 2))
    fi

    # 确保最小核心数为1（避免极端情况total_cores=1时出现0）
    if [ "${build_cores}" -lt 1 ]; then
        build_cores=1
    fi

    echo "${build_cores}"
}


function build_project()
{
    # build
    # if [ -d "${PROJECT_BUILD_DIR}" ]; then
    #     rm -rf "${PROJECT_BUILD_DIR}"
    # fi
    mkdir -p "${PROJECT_BUILD_DIR}"
    cd ${PROJECT_BUILD_DIR}
    cmake $CROSS_FLAG $CMAKE_ARGS ${PROJECT_DIR}

    make -j${NUM_CORE}

    echo ">>>>Installing to ${INSTALL_DIR} ..."
    make install > "$PROJECT_BUILD_DIR/install.log" 2>&1
    echo ">>>>Install completed."
}

function has_joint_state_data()
{
    local prefix="$1"
    [ -f "${prefix}/include/crb_ros_msg/crb_ros_msg/msg/joint_state_data.hpp" ] || \
        [ -f "${prefix}/include/crb_ros_msg/msg/joint_state_data.hpp" ]
}

function configure_crb_ros_msg()
{
    HL_MOTION_PREFIX="${HL_MOTION_PREFIX:-/workspace/hl_motion}"
    local compat_source="${PROJECT_DIR}/cmake/crb_ros_msg_compat"
    local compat_install="${BUILD_DIR}/crb_ros_msg_${SOFTWARE_ARCH}"

    if [ -d "${HL_MOTION_PREFIX}/share/crb_ros_msg/cmake" ] && \
       has_joint_state_data "${HL_MOTION_PREFIX}" && \
       [ "${HL_MOTION_PREFIX}" != "${compat_install}" ]; then
        echo "使用外部 crb_ros_msg: ${HL_MOTION_PREFIX}"
    else
        # rosidl in the Humble cross image mishandles non-ASCII paths while
        # parsing generated IDL tuples, so keep its temporary build path ASCII.
        local compat_build="/tmp/casbot_crb_ros_msg_${SOFTWARE_ARCH}_build"

        echo "构建随项目提供的 JointStateData 兼容接口"
        cmake ${CROSS_FLAG} \
            -S "${compat_source}" \
            -B "${compat_build}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX="${compat_install}"
        cmake --build "${compat_build}" --parallel "${NUM_CORE}"
        cmake --install "${compat_build}"
        HL_MOTION_PREFIX="${compat_install}"
    fi

    export CMAKE_PREFIX_PATH="${HL_MOTION_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
    CMAKE_ARGS="${CMAKE_ARGS} -Dcrb_ros_msg_DIR=${HL_MOTION_PREFIX}/share/crb_ros_msg/cmake"
}

function build_deb()
{
    # build deb package
    PKG_NAME="$1"          # 参数1：软件包名称
    PKG_VERSION="$2"       # 参数2：版本号
    PKG_ARCH="$3"          # 参数3：架构（如 amd64、arm64 等）
    SOURCE_DIR="$4"       # 参数4：程序目录路径（需存在且可读取）
    INSTALL_PATH="$5"      # 参数5：安装路径（如 /usr/bin、/opt/app 等）
    OUTPUT_DIR="$6"        # 参数6：输出目录（如 ./output_deb 等）
    DEB_ARCH="x86_64"
    if [ $# -ne 6 ]; then
        echo "Usage: build_deb <pkg_name> <pkg_version> <pkg_arch> <source_dir> <install_path> <output_dir>"
        return 1
    fi
    PKG_MAINTAINER="embedded <embedded@casbot.dev>"  # 维护者（可改为参数）
    PKG_DESCRIPTION="Automatically packaged application"  # 描述（可改为参数）
    DEST_DIR="/tmp/${PKG_NAME}/debian_build"  # 临时构建目录

    # 检查PKG_ARCH是否合法
    if [[ "$PKG_ARCH" != "amd64" && "$PKG_ARCH" != "arm64" ]]; then
        echo "错误：不支持的架构 $PKG_ARCH, 请使用 amd64、arm64"
        exit 1
    fi
    if [ "$PKG_ARCH" == "amd64" ]; then
        DEB_ARCH="x86_64"
    elif [ "$PKG_ARCH" == "arm64" ]; then
        DEB_ARCH="aarch64"
    fi

    # 检查程序文件目录是否存在
    if [ ! -d "$SOURCE_DIR" ]; then
        echo "错误：程序目录 $SOURCE_DIR 不存在！"
        exit 1
    fi
    
    # 创建输出目录
    mkdir -p "$OUTPUT_DIR"

    # 清理旧构建目录并创建新结构
    # rm -rf "$DEST_DIR"
    mkdir -p "$DEST_DIR/DEBIAN"
    mkdir -p "$DEST_DIR$INSTALL_PATH"  # 基于用户指定的安装路径创建目录

    # 复制程序文件到临时目录（保持安装路径结构）
    rsync -a --delete "$SOURCE_DIR"/* "$DEST_DIR$INSTALL_PATH/"

    # version.desc 文件处理
    BUILDER=${BUILD_ENV}
    BUILDDATE=$(date +"%Y%m%d-%H%M%S")

    cat > "$DEST_DIR$INSTALL_PATH/version.desc" <<EOF
version: $PKG_VERSION
builddate: $BUILDDATE
builder: $BUILDER
gitbranch: $G_GITBRANCH
gitcommitid: $G_GITCOMMITID
EOF

    # 生成控制文件（DEBIAN/control）
    cat << EOF > "$DEST_DIR/DEBIAN/control"
Package: $PKG_NAME
Version: $PKG_VERSION
Architecture: $PKG_ARCH
Maintainer: $PKG_MAINTAINER
Description: $PKG_DESCRIPTION
Depends: libc6 (>= 2.27)
Section: utils
Priority: optional
EOF
    # 安装前
    cat << EOF > "$DEST_DIR/DEBIAN/preinst"
#!/bin/bash
set -e

EOF
    chmod 755 "$DEST_DIR/DEBIAN/preinst"

    # 安装后
    cat << EOF > "$DEST_DIR/DEBIAN/postinst"
#!/bin/bash
set -e

# 记录安装的日志
echo "Installed $PKG_NAME version $PKG_VERSION on $(date +"%Y-%m-%dT%H:%M:%SZ")" >> /var/log/${PKG_NAME}_install.log
EOF
    chmod 755 "$DEST_DIR/DEBIAN/postinst"
    
    # 卸载前
    cat << EOF > "$DEST_DIR/DEBIAN/prerm"
#!/bin/bash
set -e
EOF
    chmod 755 "$DEST_DIR/DEBIAN/prerm"

    # 卸载后
    cat << EOF > "$DEST_DIR/DEBIAN/postrm"
#!/bin/bash
set -e
EOF
    chmod 755 "$DEST_DIR/DEBIAN/postrm"

    # 打包为.deb文件
    OUTPUT_DEB="${PKG_NAME}-${PKG_VERSION}-${PKG_ARCH}-${BUILDDATE}.deb"
    export DPKG_DEB_COMPRESS=gzip
    dpkg-deb -Zgzip --build "$DEST_DIR" "$OUTPUT_DIR/$OUTPUT_DEB"

    # 清理临时目录
    # rm -rf "$DEST_DIR"

    echo "打包完成：$OUTPUT_DEB"
    echo "可通过以下命令安装: sudo dpkg -i $OUTPUT_DIR/$OUTPUT_DEB"
}

function get_arch() {
    local arch_raw
    local target_arch

    # 如果 BUILD_ENV 非空，则优先使用其值并校验
    if [ -n "${BUILD_ENV}" ]; then
        case "${BUILD_ENV}" in
            arm64|amd64)
                target_arch="${BUILD_ENV}"
                ;;
            *)
                echo "错误: BUILD_ENV 必须为 arm64 或 amd64, 当前值为 '${BUILD_ENV}'"
                exit 1
                ;;
        esac
    else
        # 从系统获取原始架构并转换
        arch_raw=$(arch)
        case "${arch_raw}" in
            x86_64)
                target_arch="amd64"
                ;;
            aarch64)
                target_arch="arm64"
                ;;
            *)
                echo "错误：不支持的系统架构 '${arch_raw}'，仅支持 x86_64(amd64)和 aarch64(arm64)"
                exit 1
                ;;
        esac
    fi

    echo "${target_arch}"
}

function main()
{
    # 主流程开始
    NUM_CORE=$(get_build_core_num)
    echo "构建程序核心数: ${NUM_CORE}"

    if [ $# -eq 0 ]; then
        ENABLE_PACKAGE="false"
        SOFTWARE_ARCH=$(get_arch)
        SOFTWARE_NAME="casbot-rl-controller"
        SOFTWARE_VERSION="0.0.0"
    elif [ $# -eq 2 ]; then
        ENABLE_PACKAGE="true"
        SOFTWARE_NAME="$1"          # 参数1：软件包名称
        SOFTWARE_VERSION="$2"       # 参数2：版本号
        SOFTWARE_ARCH=$(get_arch)
    elif [ $# -eq 3 ]; then
        ENABLE_PACKAGE="true"
        SOFTWARE_NAME="$1"          # 参数1：软件包名称
        SOFTWARE_VERSION="$2"       # 参数2：版本号
        SOFTWARE_ARCH="$3"          # 参数3：架构（如 amd64、arm64 等）
    else
        echo "Usage: $0"
        echo "Usage: $0 <name> <version>"
        echo "Usage: $0 <name> <version> <arch>"
        echo "   eg: $0"
        echo "       $0 casbot-motion 0.1.0"
        echo "       $0 casbot-motion 0.1.0 arm64"
        echo "       $0 casbot-motion 0.1.0 amd64"
        exit 1
    fi
    
    # SOFTWARE_ARCH 判断是否为arm64或amd64
    if [[ "$SOFTWARE_ARCH" != "arm64" && "$SOFTWARE_ARCH" != "amd64" ]]; then
        echo "ARCH is error: $SOFTWARE_ARCH"
        exit 1
    fi

    INSTALL_DIR="${BUILD_DIR}/install_${SOFTWARE_ARCH}"
    CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR"
    CROSS_FLAG=""

    if [ "$SOFTWARE_ARCH" == "arm64" ]; then
        CROSS_FLAG="-DCMAKE_CUSTOM_CROSS=ON -DCMAKE_TOOLCHAIN_FILE=${PROJECT_DIR}/cmake/toolchain-sysroot.cmake"
    elif [ "$SOFTWARE_ARCH" == "amd64" ]; then
        CROSS_FLAG=""
    fi

    configure_crb_ros_msg

    SOFTWARE_OUTPUT_DIR="${BUILD_DIR}/output_deb"
    PROJECT_BUILD_DIR="${BUILD_DIR}/build_$SOFTWARE_ARCH"

    # Customer source releases do not include the internal Git repository.
    if [ -e "${PROJECT_DIR}/.git" ] && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        G_GITBRANCH=$(git rev-parse --abbrev-ref HEAD)
        G_GITCOMMITID=$(git rev-parse HEAD)
        GITCOMMIT_STATUS=$(git status --porcelain)
        if [ -n "$GITCOMMIT_STATUS" ]; then
            G_GITCOMMITID="${G_GITCOMMITID}-dirty"
        fi
        if ! git show-ref --verify --quiet refs/remotes/origin/"$G_GITBRANCH"; then
            G_GITCOMMITID="${G_GITCOMMITID}-unpush"
        elif [ -n "$(git log origin/"$G_GITBRANCH"..HEAD)" ]; then
            G_GITCOMMITID="${G_GITCOMMITID}-unpush"
        fi
    else
        G_GITBRANCH="source-release"
        G_GITCOMMITID="source-release"
    fi

    echo ">>>>>>>>>>>>>>>>"
    echo "build core: $NUM_CORE"
    echo "name: $SOFTWARE_NAME"
    echo "version: $SOFTWARE_VERSION"
    echo "arch: $SOFTWARE_ARCH"
    echo "INSTALL_DIR: $INSTALL_DIR"
    echo "install_path: $SOFTWARE_INSTALL_PATH"
    echo "project_build_dir: ${PROJECT_BUILD_DIR}"
    echo "output_dir: $SOFTWARE_OUTPUT_DIR"
    echo "git branch: $G_GITBRANCH"
    echo "git commit id: $G_GITCOMMITID"
    echo ">>>>>>>>>>>>>>>>"

    # 验证版本号格式（严格的三段式：数字.数字.数字，如 1.0.0、1.0.110、1.2.1）
    if ! [[ "$SOFTWARE_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Error: Invalid version format. Must be 'x.y.z' (e.g., 1.0.0, 1.0.110, 1.2.1)"
        exit 1
    fi

    # SOFTWARE_ARCH 验证
    if [[ "$SOFTWARE_ARCH" != "amd64" && "$SOFTWARE_ARCH" != "arm64" ]]; then
        echo "Error: Unsupported architecture $SOFTWARE_ARCH. Please use amd64 or arm64."
        exit 1
    fi

    # if [ -d $BUILD_DIR ]; then
    #     rm -rf $BUILD_DIR
    # fi
    mkdir -p "$BUILD_DIR"

    if [ -d $INSTALL_DIR ]; then
        rm -rf $INSTALL_DIR
    fi
    mkdir -p "$INSTALL_DIR"

    build_project

    if [ "${ENABLE_PACKAGE}" == "true" ]; then
        build_deb "$SOFTWARE_NAME" "$SOFTWARE_VERSION" "$SOFTWARE_ARCH" "$INSTALL_DIR" "$SOFTWARE_INSTALL_PATH" "$SOFTWARE_OUTPUT_DIR"
    fi
}

main "$@"
