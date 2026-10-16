# CASBOT02 mjlab loco

CASBOT02 平地速度跟踪训练版。完整操作说明见 [训练流程.md](训练流程.md)。

本项目提供机器人 XML/STL、PPO 训练、play、ONNX 导出和 MuJoCo sim2sim。
训练任务：`Casbot02-Loco-Teacher-Flat`；actor 输入196维，critic 输入208维，输出12个腿部关节动作。
手臂随左右膝关节差自动摆动，不占策略动作维度。


## 文件导航

| 文件 | 用途 |
|---|---|
| `src/wbc_mjlab/casbot02_config.py` | 奖励、观测、速度课程、随机化、环境数量 |
| `src/wbc_mjlab/casbot02_constants.py` | 机器人结构路径、默认姿态、电机参数、动作缩放 |
| `src/wbc_mjlab/casbot02_rl_cfg.py` | PPO 网络、学习率、迭代数、对称增强 |
| `src/wbc_mjlab/CASBOT02_7dof/` | 当前训练机器人 XML 和引用的 STL |
| `train.sh` / `play.sh` | 训练和播放入口 |
| `scripts/export_casbot02_onnx.py` | 导出包含观测归一化的 ONNX |
| `scripts/sim2sim_casbot02_loco_onnx.py` | 独立 MuJoCo ONNX 播放与 CSV 记录 |
| `SOURCE_MANIFEST.json` | 提取来源版本及原始文件哈希 |




