#!/usr/bin/env bash
# 获取当前的执行脚本名称
arg0=${BASH_SOURCE[0]}
if [ ! $arg0 ];
then
arg0=$0
fi

# 计算当前的目录，根据当前目录计算命令所在目录
if [ -h $arg0 ]
then
link_path=`readlink $arg0`
else
link_path=$arg0
fi

# 将当前目录设置为 root 目录
export RL_MOTION_ROOT=$(cd `dirname ${link_path}`; pwd)
export rl_motion_ROOT=$RL_MOTION_ROOT


# 将第三方库添加到查找路径
LIB_ROOT="${RL_MOTION_ROOT}/lib"
for lib_dir in "${LIB_ROOT}"/*
do
    if [ -d ${lib_dir} ];
    then
        export LD_LIBRARY_PATH=${lib_dir}:$LD_LIBRARY_PATH
    fi
done
unset lib_dir
export LD_LIBRARY_PATH=$RL_MOTION_ROOT/lib:$LD_LIBRARY_PATH

export PATH=$PATH:$RL_MOTION_ROOT/bin