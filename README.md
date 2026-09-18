# PICO → Marvin 双臂遥操与数据采集

本项目通过 PICO 手柄遥操 Marvin 双臂，并采集 PICO、机器人、DAS 夹爪和双目相机数据。项目包含 ROS 2 数据链路、MCAP 录制与后处理工具，以及本地 Web 控制台。

> Web 停止按钮和程序退出不是急停。实机运行前必须确认物理急停、关节映射、机器人型号和工作区安全；发生异常运动时立即使用物理急停。

## 快速开始

运行环境为 Ubuntu 22.04、Python 3.10 和 ROS 2 Humble。首次安装请阅读[部署指南](docs/deployment/部署指南.md)。

```bash
source /opt/ros/humble/setup.bash
uv sync
uv run teleop-web --self-test
uv run teleop-web
```

浏览器访问 <http://127.0.0.1:4173>。远程使用 NUC 时可建立 SSH 隧道：

```bash
ssh -L 4173:127.0.0.1:4173 zxcx@192.168.1.11
```

运行前检查并按现场环境修改：

- `config/collection.json`：机器人、采集、输出和运行时配置；
- `config/das_gripper.example.json`：DAS 夹爪、串口和相机配置；
- `config/collection.nuc.json`：NUC 的 CPU 亲和性配置。

验证配置：

```bash
uv run teleop-collect --config config/collection.json --print-effective-config
```

## 常用入口

```bash
uv run teleop-web                 # Web 控制台
uv run teleop-collect --help      # 完整采集流程
uv run teleop-hardware --help     # 实机遥操
uv run teleop-postprocess --help  # Episode 后处理
uv run teleop-validate --help     # Episode 校验
```

完整参数、标定、复位和数据迁移命令见[操作指南](docs/operations/操作指南.md)。

## 项目结构

| 路径 | 内容 |
| --- | --- |
| `src/xr_marvin_teleop/` | 控制器、设备适配器、ROS 2、采集与 CLI |
| `config/` | 采集、DAS 和相机配置 |
| `legacy/ros2_ws/` | ROS 2 消息与工作区 |
| `ui/` | Web 控制台前端和架构说明 |
| `tests/` | 无硬件单元测试与实机检查 |
| `tools/` | 延迟日志分析工具 |
| `vendor/` | 本地厂家 SDK；部分内容不纳入 Git |

控制与采集的数据流见[控制链路](docs/deployment/控制链路.md)，部署验收项见[检查清单](docs/deployment/check.md)。

## 测试

```bash
uv run python -m unittest discover -s tests -v
node ui/test.mjs
```

部分测试需要 ROS 2、MuJoCo、厂家 SDK 或已连接硬件；跳过原因会由测试输出说明。
