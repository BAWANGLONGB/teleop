# 天机机械臂调试

> 更新日期：2026-08-28
>
> 整理来源：[飞书《天机机械臂调试》](https://dcnt8jf6zo2j.feishu.cn/docx/ZNdwdGGWWoZhhjx0ydRc7Hzrn4b)及本地 SDK 资料。
>
> 安全边界：本文包含真机连接和运动命令。任何实机操作前，必须确认准确机型、左右臂映射、Tool、关节方向、物理急停和空载工作区。优先使用仿真，不得将 Demo 的默认参数直接用于真机。

## 1. 参考资料

- [Marvin PlatformEN 软件使用说明](天机Marvin系列_Marvin%20PlatformEN软件使用说明260804.md)
- [Marvin PlatformEN 原始 PPTX](天机Marvin系列_Marvin%20PlatformEN软件使用说明260804.pptx)
- [Marvin 急停功能启用方法](天机Marvin系列_急停功能启用方法说明251203.md)
- [Marvin 系列机器人使用说明书](tj_fx_robot-master/Marvin系列机器人使用说明书_1121200066_V2.2.pdf)
- [JMDT 驱动器报警故障说明书](tj_fx_robot-master/JMDT驱动器报警故障和原因措施说明书_1121200064_V1.0.pdf)
- [SDK 总体说明](tj_fx_robot-master/README.md)
- [Python 控制接口说明](tj_fx_robot-master/python_doc_contrl.md)
- [Python 运动学接口说明](tj_fx_robot-master/python_doc_kine.md)

## 2. 工程结构

```text
tj_fx_robot-master/
├── MarvinPlatform_EN/       # Linux/Windows 上位机及源码
├── SDK_PYTHON/             # Python 控制和运动学封装
├── contrlSDK100343/        # 控制 SDK 源码
├── kinematicsSDK/          # 运动学 SDK 源码
├── interferenceCheck/      # 干涉/碰撞检测库
├── DEMO_PYTHON/            # Python 示例
├── DEMO_C++/               # C++ 示例
├── CommonConfig/           # 各机型运动学配置
├── robot.ini               # 控制器/机型参数
├── tools_cfg.json          # Tool 运动学和动力学参数
└── TargetIP.CFG            # 目标控制器地址
```

基本调用链路：

```text
Python Demo / MarvinPlatform
  → SDK_PYTHON
  → libMarvinSDK.so（控制）/ libKine.so（运动学）
  → UDP
  → Marvin 控制器
```

## 3. 参数辨识

### 3.1 原始数据备份

- 辨识数据：[`tj_robot.zip`](tj_robot.zip)
- 备份前记录机型、控制器版本、SDK 版本、左右臂、负载状态和采集日期。
- 原始数据、处理脚本和辨识结果分开保存，不覆盖原始数据。

### 3.2 简智夹爪负载

夹爪辨识结果已保存到 Git 中，当前本地文件为：

- [`tools_cfg.json`](tools_cfg.json)
- [`tj_fx_robot-master/tools_cfg.json`](tj_fx_robot-master/tools_cfg.json)

使用前必须核对左/右臂 Tool 索引、质量、质心、惯量和法兰安装方向。修改辨识结果时应保留旧版本并记录差异。

## 4. 仓库与基本调试

### 4.1 仓库

- 内网仓库：<http://192.168.121.142/changxu/tj_fx_robot/-/tree/master>
- 本地副本：`/home/zxcx/TeleOp/机械臂调试/tj_fx_robot-master`

### 4.2 网络连接

1. 用独立网线连接机械臂控制器。
2. 控制器默认目标地址为 `192.168.1.190`，以现场设置为准。
3. 主机有线网卡设为同网段的不同地址，例如 `192.168.1.100/24`；不得把主机设为 `192.168.1.190`。
4. 连接前检查路由与连通性：

   ```bash
   ip -brief -4 addr show scope global
   ip route get 192.168.1.190
   ping -c 4 192.168.1.190
   ```

### 4.3 编译 Linux 动态库

```bash
cd /home/zxcx/TeleOp/机械臂调试/tj_fx_robot-master
chmod +x marvinSDK_ubuntu_100343.sh
./marvinSDK_ubuntu_100343.sh
```

脚本会编译并替换 `libMarvinSDK.so`、`libKine.so` 和 `libInterfCheck.so`。重新编译前先确认 SDK/控制器版本匹配，不要在现场验收中临时替换库。

### 4.4 启动 MarvinPlatform

```bash
cd /home/zxcx/TeleOp/机械臂调试/tj_fx_robot-master
python3 MarvinPlatform_EN/ui_EN.py
```

连接、状态切换、Tool 配置、拖动、辨识和故障处理见 [Marvin PlatformEN 软件使用说明](天机Marvin系列_Marvin%20PlatformEN软件使用说明260804.md)。

### 4.5 Python 双臂多段直线 Demo

入口：[`DEMO_PYTHON/showcase_pln_multi_segment_linear_two_arms_classes.py`](tj_fx_robot-master/DEMO_PYTHON/showcase_pln_multi_segment_linear_two_arms_classes.py)

先运行仿真：

```bash
cd /home/zxcx/TeleOp/机械臂调试/tj_fx_robot-master
python3 DEMO_PYTHON/showcase_pln_multi_segment_linear_two_arms_classes.py sim
```

`real` 后端会向真实机械臂发送运动命令：

```bash
python3 DEMO_PYTHON/showcase_pln_multi_segment_linear_two_arms_classes.py real
```

> **警告：** 该 Demo 的开发默认值包含 `velocity=100` 和 `acceleration=100`，且轨迹幅度不代表已通过本机现场验收。未完成机型、映射、Tool、急停和逐轴低速验证时，不得运行 `real`。

## 5. 坐标系说明

### 5.1 输入点位

笛卡尔点格式：

```text
[X, Y, Z, A, B, C]
```

| 分量 | 含义 | 单位 |
| --- | --- | --- |
| `X, Y, Z` | 位置 | mm |
| `A` | 绕 X 轴的 roll | degree |
| `B` | 绕 Y 轴的 pitch | degree |
| `C` | 绕 Z 轴的 yaw | degree |

旋转顺序：

$$
R = R_z(C) R_y(B) R_x(A)
$$

SciPy 验证代码：

```python
from scipy.spatial.transform import Rotation

R = Rotation.from_euler(
    "xyz",
    [A, B, C],
    degrees=True,
).as_matrix()
```

左侧轨迹在左臂自身的 SDK `Base_L` 坐标系下计算，右侧轨迹在右臂自身的 SDK `Base_R` 坐标系下计算。不要把两侧 TCP 点直接当作同一基坐标系的数值。

### 5.2 Tool、UserFrame 与 TCP

SDK 初始化时 `Tool` 和 `UserFrame` 默认为单位变换，可通过以下接口修改：

```python
set_tool_kine(...)
set_user_frame(...)
```

目标 TCP 语义：

$$
\text{TargetTCP} = \text{UserFrame} \cdot \text{FlangeTip}(q) \cdot \text{Tool}
$$

- 左臂 `FlangeTip`：`TCP_Link_L` 相对 `Base_L`。
- 右臂 `FlangeTip`：`TCP_Link_R` 相对 `Base_R`。
- 默认 Tool 为单位变换时，TCP 与 `TCP_Link_L/R` 重合。

## 6. 常见问题

### 6.1 连接成功但机械臂进入错误状态

已观察到的现象：

- 弹窗显示 `robot connection successful`；
- 同时提示 `tool information not set for robot`；
- 左右臂 `status=100`；
- 报错 `arm error 13` / `emergency stop`。

排查顺序：

1. 停止运动命令，确认机械臂和工作区安全。
2. 检查物理急停是否已按 [急停功能启用方法](天机Marvin系列_急停功能启用方法说明251203.md) 完成接线和配置。
3. 确认控制器 Tool 信息已正确设置，且与实际末端负载一致。
4. 在 MarvinPlatform 中读取左右臂错误码和状态，按说明复位，不要在不明原因时反复清错。

飞书原记录的临时规避方式是把 `robot.ini` 中的 `UseEMG` 设为 `0`。这会绕过急停链路检查，只能在断开伺服、禁止运动输出的诊断场景下，由供应商或负责人明确授权后使用。

截至本次整理，本地 [`tj_fx_robot-master/robot.ini`](tj_fx_robot-master/robot.ini) 的实际值为 `UseEMG=0`；因此该文件当前**不符合带使能真机运行的急停门禁**。本文只记录该风险，未擅自修改控制器配置。

> **禁止：** `UseEMG=0` 不是可验收的修复方案。进入任何带使能运动前，必须恢复急停监测、按官方文档完成配置，并实测物理急停确实能中断运动。

## 7. SDK 运动学与规划备忘

### 7.1 SDK DH 坐标系与原版 URDF Link 的差异

以左臂为例，固定关系 $C$ 如下：

| SDK 坐标系 | 原版 URDF 坐标系 | 固定关系 $C$ |
| --- | --- | --- |
| `JointPG[0]` | `Link1_L` | $I$，重合 |
| `JointPG[1]` | `Link2_L` | $I$，重合 |
| `JointPG[2]` | `Link3_L` | $Trans(0,0,329\ \text{mm})$ |
| `JointPG[3]` | `Link4_L` | $R_z(180°)$ |
| `JointPG[4]` | `Link5_L` | $I$，重合 |
| `JointPG[5]` | `Link6_L` | $R_z(90°)$ |
| `JointPG[6]` | `Link7_L` | $R_z(90°)$ |
| `FlangeTip` | `TCP_Link_L` | $I$，重合 |
| `TCP` | `TCP_Link_L` | 默认 Tool 下为 $I$，重合 |

右臂不应直接照搬左臂结论，使用前应结合对应 URDF、机型配置和 FK 结果单独复核。

### 7.2 IK 解算

核心流程：解腕点 → 确定肘关节（Joint4）→ 确定臂角 → 解算其余关节。

臂角控制逻辑：

```text
RefJoint / ZSPPara
  → 普通 IK 生成基准臂平面缓存
  → 叠加 ZSP_Angle
  → 在保持 TCP 的前提下得到新臂角解
```

| 输入 | 含义 | 作用 |
| --- | --- | --- |
| `RefJoint` | 用参考关节角隐式定义参考臂平面 | `ZSPType=0` 时选择最接近参考关节角的解 |
| `ZSPPara=[x,y,z,0,0,0]` | 显式提供参考臂平面方向 | `ZSPType=1` 时选择臂平面最接近该方向的解 |
| `ZSP_Angle` | 相对当前参考臂平面旋转 | `ik_nsp()` 保持 TCP 不变并主动改变臂角 |

异常与边界处理：

| 情况 | 检测方式 | SDK 处理 |
| --- | --- | --- |
| 超过最大工作半径 | `ablen + 0.001 >= cart_len` | `IsOutRange=true`、`IsDeg[3]=true`，返回 `RefJoint`，函数返回 `false` |
| Joint4 构型边界 | Joint4 接近 `m_J4_Bound ±0.1°` | 按 Joint4 奇异处理并返回 `false` |
| Joint2 奇异 | `J123` 的 ZYZ 中间角接近 0 | `IsDeg[1]=true`，用 `RefJoint` 固定不可辨识角 |
| Joint6 奇异（SRS） | `J567` 的 ZYZ 中间角接近 0 | `IsDeg[5]=true`，用 `RefJoint` 补齐不可辨识角 |
| Joint6 奇异（CCS） | ZYX 分解中 $\cos(q_6)$ 接近 0，即约 $±90°$ | 分解函数返回 `false` |
| 普通关节越界 | 与 `m_JLmtPos_N/P` 比较 | `IsJntExd=true`，设置各轴 `JntExdTags`，但 IK 仍可返回 `true` |
| CCS Joint6/Joint7 耦合越界 | 按 Joint6 正负选择二次曲线计算 Joint7 动态限位 | 更新 Joint7 允许范围并设置越界标记 |

> 调用者必须同时检查函数返回值和 `IsOutRange` / `IsDeg` / `IsJntExd` / `JntExdTags`；不能只因 IK 返回 `true` 就认定结果可下发。

### 7.3 规划方式

| 类型 | 输入 | 是否生成轨迹 | ZSP 控制 | 输出 |
| --- | --- | --- | --- | --- |
| `ik` | 单个 TCP | 否 | `RefJoint` / `ZSPPara` | 单点关节解 |
| `ik_nsp` | angle + 上次 IK 状态 | 否 | `ZSP_Angle` | 同一 TCP 的新关节解 |
| `movL` / `movLA` | 起终 TCP | 单段直线 | 固定 `ZSPType=0` | 文件/点集 |
| `multi_movL` | 多个 TCP | 多段直线+衔接 | 支持 `ZSPType` / `ZSPPara` | 点集 |
| `movL_KeepJ(A)` | 起终关节角 | 单段直线 | 插值 NSP，越界时自动调 angle | 文件/点集 |
| `mov_target` | 起终 TCP | 直线优先、逐级降级 | 间接使用 | 点集 |

### 7.4 规划执行接口

| Python 接口 | C 导出接口 | 底层实现 | 用途 |
| --- | --- | --- | --- |
| `setPln_joint("A/B", ...)` | `OnSetPlnJoint_A/B` | `CRobot::OnSetPlnJoint_A/B` | 单臂关节规划执行 |
| `setPln_joint_AB(...)` | `CoRunPlnJoint` | `CRobot::OnSetPlnJoint_AB` | 双臂关节规划、同时启动 |
| `setPln_Cart("A/B", pset)` | `OnSetPlnCart_A/B` | `CRobot::OnSetPlnCart_A/B` | 单臂预计算点集执行 |
| `setPln_Cart_AB(psetA, psetB)` | `CoRunPlnCart` | `CoRunPlnCart` 内部直接上传 | 双臂点集、同时启动 |
| `stopRunPln_joint("A/B")` | `OnStopPlnJoint_A/B` | `OnStopPlnJoint_interA/B` | 停止单臂规划轨迹 |
| `stopPln_AB()` | `CoStopPln` | 同帧加入 A/B 停止命令 | 双臂同时停止 |
| `run_pln_joint("A/B", ...)` | `RunPlnJoint` | 最终调用 `CRobot::OnSetPlnJoint_A/B` | 新版单臂高级封装 |
| `run_pln_cart("A/B", pset)` | `RunPlnCart` | 最终调用 `CRobot::OnSetPlnCart_A/B` | 新版单臂高级封装 |
| `stop_pln("A/B")` | `StopPln` | 最终调用单臂停止接口 | 新版单臂停止封装 |

### 7.5 末端负载/外力估计

示例：[`DEMO_PYTHON/showcase_jointsTorque2EefTorque.py`](tj_fx_robot-master/DEMO_PYTHON/showcase_jointsTorque2EefTorque.py)

该脚本订阅关节位置和 `est_joint_force`，计算当前 Jacobian，再从关节外力估计末端六维力/力矩。使用时需要特别确认：

- `arm='A'` / `arm='B'` 与 `idx=0` / `idx=1` 一致；
- `calculate_config_file` 与真机准确机型、版本一致；
- Tool 负载和零偏已正确设置；
- 接近奇异位形时 Jacobian 求解可能失败或放大噪声；
- 该结果是估计值，不得未经标定就当作功能安全或精密测力依据。

## 8. 真机调试最小检查清单

1. 确认机型、左右臂、SDK 版本和配置文件。
2. 确认主机与控制器同网段且 IP 不冲突。
3. 确认 Tool 与实际末端负载一致。
4. 确认物理急停已启用并实测有效，`UseEMG` 未被绕过。
5. 先连接和订阅状态，确认 A/B 数据持续更新、无错误码且机械臂静止。
6. 先仿真，再单臂、低速、小范围、空载验证，最后才考虑双臂。
7. 运行时保持急停观察员在场；出现失控趋势、碰撞风险或进程无响应时，先按物理急停。
8. 正常结束时下伺服/复位到安全状态并释放 SDK，保存命令、参数、日志和异常现象。
