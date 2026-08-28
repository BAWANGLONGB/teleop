# TeleOp 工作区

本仓库汇总天机机械臂、XRoboToolkit/PICO 遥操、MuJoCo 仿真及相关 SDK 示例。
主要遥操工程位于 [`xrobotoolkit-marvin-teleop/`](xrobotoolkit-marvin-teleop/)。

## 快速开始

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xrobotoolkit-marvin-teleop
```

先启动 XRoboToolkit PC Service，再在 PICO 中配置主机局域网 IPv4，开启
`Head`、`Controller` 和 `Send`，然后运行 MuJoCo 仿真：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
```

## 开发与操作文档

- [简洁版开发流程与操作指南](xrobotoolkit-marvin-teleop/docs/简洁版开发流程与操作指南.md)：日常修改、测试、仿真和实机启动清单。
- [环境部署与 PICO 联调流程](xrobotoolkit-marvin-teleop/docs/XRoboToolkit环境部署与PICO联调流程.md)：PC Service、PICO 配网、双网卡及故障排查 SOP。
- [Marvin 控制与测试调参指南](xrobotoolkit-marvin-teleop/docs/Marvin机械臂控制与测试调参指南.md)：控制参数、限幅、回位和实机验收。
- [开发计划](xrobotoolkit-marvin-teleop/docs/XR-Robotics天机双臂VR遥操开发计划.md)：设计依据、参数推导和未关闭风险。

## 安全要求

自动测试和 MuJoCo 通过不等于实机验收通过。首次接入真机前，必须确认机械臂型号、
A/B 关节映射、Tool 参数、网络路由和物理急停，并由现场观察员执行低速单臂测试。
未经确认不要运行带 `--enable-hardware` 的入口；出现失控趋势时优先按物理急停。

## 验证

```bash
cd /home/zxcx/TeleOp/xrobotoolkit-marvin-teleop
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q tests/test_marvin_hardware.py tests/test_marvin_mujoco_model.py
```

根项目使用独立 Git 元数据目录（平台挂载的 `.git` 不可写），执行 Git 命令时请使用：

```bash
git --git-dir=/home/zxcx/TeleOp/.teleop-git --work-tree=/home/zxcx/TeleOp status
```
