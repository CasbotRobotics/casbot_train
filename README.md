# rl_controller 客户源码包

用于 CASBOT02 的 ROS 2 策略部署。依赖 Ubuntu 22.04 / ROS 2 Humble，机器人端需已安装并配置匹配的 `hl_motion`。

## 当前模型与观测

- 模型：`config/model/2026-10-15_18-42-34_casbot02_loco_teacher.onnx`
- 网络输入：196 维，输出：12 维腿部 action。
- 输入包含 45 维基础观测的 4 帧历史，以及 4 维相位的 4 帧历史。
- 50 Hz 策略推理，1 kHz 控制循环；具体开关和参数见 `config/rl_config.yaml`。
- 其他手臂关节的目标值、增益和肩关节摆动逻辑见 `rl_controller/rl_interface.cpp`。


## 部署到机器人

登录orin并在workspace下面解压motion和rlcontroller的压缩包 `scripts/install_remote.sh` 上传 ARM64 构建结果，默认地址为 `nvidia@172.16.0.10`，运行时询问密码；机器人需已允许该账号操作 `/workspace/rl_controller`。

本分支不包含 `casbot-motion` 安装包、策略 ONNX 和 ONNX Runtime 动态库，这些文件需要另行准备。

```bash
# 登录
ssh -X nvidia@172.16.0.10
sudo su 
sudo dpkg -i casbot-motion_2.2.37_arm64.deb
sudo dpkg -i rlcontrollerxxx
```

机器人端启动：

```bash
source /opt/ros/humble/setup.bash
source /workspace/hl_motion/setup.bash
source /workspace/rl_controller/setup.bash
/workspace/rl_controller/bin/rl_controller_runner
```

实际加载的模型和配置路径为 `/workspace/rl_controller/config/` 下对应文件；替换模型时须保持观测排列和维度匹配。

## 代码与验证工具

- `rl_controller/`：观测、相位历史、ONNX 推理、目标关节处理。
- `common/`、`src/`：关节定义、数据转换及 ROS 2 节点。
- `cmake/`：交叉编译配置和消息兼容接口。
- `scripts/`：环境加载和部署辅助脚本。
- `sim2sim_casbot02_loco_onnx(1).py`：观测对齐参考；完整仿真还需原训练工程的 `mjlab`、`wbc_mjlab` 等依赖。
- `joint_state_new.py`：关节数据诊断工具。
- `tests/`：离线观测对齐验证，只需 C++17 编译器和 Python numpy，不连接机器人。

```bash
g++ -std=c++17 -I . tests/loco_observation_probe.cpp -o /tmp/loco_observation_probe
python3 tests/test_loco_observation.py /tmp/loco_observation_probe
```
