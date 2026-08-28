# VR 遥操天机机械臂控制链路分析

> 本文档整理自调研与 **VR 遥操天机（TJ/TianJi）双臂机械臂控制链路** 的讨论。
> 全部结论已对照本地克隆的两个仓库源码逐条核实：`dev/roboteleop`（VR 输入侧）与 `dev/robot_teleop`（机械臂控制侧）。

---

## 1.省略

## 2. 整体控制链路（四层）

```
┌─────────────────────────────────────────────────────────────┐
│ ① PICO VR 头显（XRoboToolkit / PXREA gRPC 服务）              │
│    头 + 双手位姿、扳机、按键、人体 24+ 关节动捕数据             │
└──────────────────────────┬──────────────────────────────────┘
                           ↓ SDK 数据流（按数据源时间戳触发）
┌─────────────────────────────────────────────────────────────┐
│ ② pico_teleop_sdk_node（roboteleop，ROS2 Python 节点）        │
│    → 发布 ROS2 话题 "pico/hand_poses"（roboteleop_msgs/HandPair）│
└──────────────────────────┬──────────────────────────────────┘
                           ↓ ROS2 订阅
┌─────────────────────────────────────────────────────────────┐
│ ③ RobotController @400Hz（robot_teleop，C++ 节点）             │
│   ├─ VrTeleoperator：VR 位姿 → 双手笛卡尔目标位姿（SE3）       │
│   ├─ WBC 求解器（qpOASES）：带约束 QP 微分 IK → 14 关节目标位置 │
│   └─ FSM 状态机：TELEOP / JOINTSPACE / LeaderArm / GAMPAD ...  │
└──────────────────────────┬──────────────────────────────────┘
                           ↓ SharedDataManager（线程间共享数据）
┌─────────────────────────────────────────────────────────────┐
│ ④ SlaveHardware @100Hz                                      │
│   → MarvinSDK（天机臂官方 SDK）以太网指令 → 192.168.110.24     │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
   天机双臂（左臂 A + 右臂 B，各 7 关节）+ PGC 夹爪 ×2（串口）
```

**一句话总结**：这是一条 **"VR 笛卡尔遥操 → QP 逆运动学求关节位置 → 天机臂 SDK 关节空间阻抗模式执行"的位置型遥操链路**——上层是运动学控制（qpOASES IK），底层臂控是柔性阻抗位置控制，全链路**不直接发任何力矩指令**。

---



## 3. 各环节实现细节



### 3.1 VR 输入侧（`roboteleop/src/pico_teleop_pkg/`）

- 两个可用的取数节点：
  - `pico_teleop_node.py`：通过 **gRPC**（`PXREAService` 的 `EAServiceStub`，proto 定义在 `proto/PXREAService_pb2_grpc.py`）连接 PICO 头显的动捕服务；
  - `pico_teleop_sdk_node.py`：使用 **XRoboToolkit 官方 SDK**，读取头显/手柄位姿、扳机值、按键，以及人体关节动捕（`get_body_joints_pose/velocity/acceleration`）。
- 发布话题：`pico/hand_poses`，消息类型 `roboteleop_msgs/msg/HandPair`（头 + 左手 + 右手位姿、左右扳机、按键、握把 + `BodyJoint[]` 身体关节）。
- 发布策略：**按 SDK 数据源时间戳触发**（不是固定频率），并做零位姿帧过滤、连接质量监控（`pico_connection_quality.py`）。



### 3.2 VR → 机器人位姿映射（`vr_teleoperator.hpp` 的 `VrTeleoperator`）

- **扳机门控**：扣住扳机才激活遥操；松开即停（目标位姿=当前位姿，速度清零），松手就是安全停机。
- **增量映射**：按下扳机的瞬间记录 VR 手柄起始位姿 `t_vr_start` 和机器人末端起始位姿 `t_robot_start`，之后：
  - 位置：`target = robot_start + motion_scale · (R_vr_to_robot · Δp_vr)`
  - 姿态：`r_delta = R_vr_to_robot · (R_rel) · R_vr_to_robotᵀ`，`target.R = r_delta · robot_start.R`
- **头部参考系**：手柄位姿先转换到头显坐标系下（`t_ref.actInv(vr_hand)`）；按 **A 键**可锁定头部参考系（`t_head_latch_`）。
- 参数（`config/marvin_dual.yaml`）：`motion_scale = 1.2`；`vr_to_robot = [1,0,0, 0,0,1, 0,-1,0]`（即绕 X 轴 -90° 的坐标旋转矩阵，把 VR 坐标系转到机器人基座系）。



### 3.3 核心控制器（`controller/src/controller.cpp` 的 `RobotController`）

- ROS2 节点 `robot_controller_node`：400Hz 控制线程 + 400Hz 发布线程。
- 每周期流程：
  1. 取当前 14 关节状态（`SharedDataManager`，仿真/实机二选一）；  
  2. **正运动学**（pinocchio FK）求双手当前位姿；
  3. `VrTeleoperator::Update()` 产出双手**目标位姿**（SE3）；
  4. `wbc_solver_->Solve(q_curr, target_L, target_R)` —— **这一步就是逆运动学**：输入目标末端位姿，输出 14 个关节角 `q_next`；
  5. 写入 `target_cmd_`，由硬件层下发给臂。
- **FSM 状态机**：
  - `INIT` → 按 YAML 的 `teleop.mode` 分流（`vr` / `leader_slave` / `gamepad`）；
  - `TELEOP`：VR 模式（**Y 键进入**，**X 键回零**回 JOINTSPACE 走 3 秒插值轨迹 `target_A`）；
  - `SYNC_TO_LEADER → LeaderArm`：主臂模式——主臂（leader arm）编码器关节角 + 偏移量直接位置跟随；
  - `GAMPAD`：手柄兜底模式；
  - `IDLE`：保持当前位置（leader_slave 下 Y 键起停）。
- **夹爪**：PGC 夹爪走串口（`/dev/gripper_l_usb`、`/dev/gripper_r_usb`），握把键 **toggle 式**开/合（`setPosition(0/1000)`，`setForce(100)`），每 2 个控制周期读一次位置反馈。



### 3.4 核心求解器：带约束的 QP 微分逆运动学（`controller/src/wbc/wbc.cpp`）

**先说结论：它名叫 WBC，但并不是教科书意义上的 Whole-Body Control**（那是基于动力学模型的逆动力学求解、输出力矩 τ）。它是**速度层面的 QP 优化 IK**，只借用了 WBC 的"多任务加权 + 零空间 + 约束"框架。全程不碰动力学（无质量矩阵、不输出力矩）。证据：代码里确实有 `id_wbc.cpp`（逆动力学版），但在 `controller.cpp` 中从未初始化、从未调用。

每个 400Hz 周期求解的问题：

**决策变量**：关节速度 q̇（14 维）

**任务空间 PD 速度参考**（12 维 = 双臂末端各 6 维位姿）：

```
v_ref = kp · e − kd · v_curr      （kp_pos=20, kp_rot=15；kd_pos=kp_pos·0.3, kd_rot=kp_rot·0.2）
```

位置误差 e 用平动差，姿态误差用 `log3(R_des · R_currᵀ)`（旋转矩阵对数映射）。

**目标函数**（最小化）：

```
‖W·(J·q̇ − v_ref)‖²  +  Σ wᵢ·q̇ᵢ²  +  Σ wᵢ·q̇ᵢ·k_null·(q_nom − q)ᵢ
   ──任务跟踪───        ──关节加权──     ──零空间姿态任务（k_null=0.5）──
```

- `J`：12×14 堆叠雅可比（左手 6 行 + 右手 6 行，每周期由 pinocchio 重算）；
- `W`：任务权重 `[10,10,10,1,1,1, 10,10,10,1,1,1]`（位置跟踪优先于姿态）；
- 关节加权 `wᵢ`：默认 1e-4，肩部三关节 1e-2，**肘部附近关节 1e-6**——冗余方向上"尽量别动"；
- 零空间项把 14 轴往 `q_nominal` 舒适姿势拉——7 轴臂对 6 维任务多出的 2 个冗余自由度（双臂各 1 个）由此"驯化"。

**箱式约束**（安全性构造性保证）：

```
max(−v_max, (q_min − q)/dt) ≤ q̇ ≤ min(v_max, (q_max − q)/dt)   （v_max = 3.1416 rad/s）
```

即：关节速度不超限，且本周期积分后**数学上不可能越出关节限位**——不是检测后限幅，而是解空间本身被约束住。VR 玩家怎么猛拽手柄，解出来的轨迹都不会越限。

**奇异阻尼**（SVD 自适应）：对雅可比做 SVD，最小奇异值 < 0.05 时对角加 λ（1e-4 ~ 1e-2 连续插值）——接近奇异构型时解自动"变软"，不会关节速度爆炸。

**求解与输出**：qpOASES `SQProblem`（首次 `init`，之后 `hotstart` 热启动，nWSR=100）解出 q̇*，再用 pinocchio `integrate()` 流形积分成关节角 `q_next` 作为位置指令返回。

**QP 标准形式**（对照代码）：

```
min  ½xᵀHx + gᵀx ,  s.t. lb ≤ x ≤ ub
H = 2·(JᵀWJ) + diag(w_joint) + λI     （正定，保证唯一解）
g = −2·JᵀW·v_ref − w_joint ⊙ q̇_null  （q̇_null = k_null·(q_nom − q)）
```



### 3.5 硬件层：MarvinSDK = 天机臂官方 SDK（`hardware/slave_hardware.hpp`）

- 头文件注释直接引用 `github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK`，配置用的 URDF 就是 `tianji_dual_arms.urdf`——**Marvin 就是天机（TJ/TianJi）双臂机器人的 SDK**。
- 以太网连接 `192.168.110.24`（`OnLinkTo(192,168,110,24)`），100Hz 循环（10ms 睡眠）。
- 初始化配置（`ConfigureRobotParams`）：
  - 工具参数（tool_para：末端 100mm 偏置；tool_dyn：质量 1.8kg 等动力学参数）;
  - **关节阻抗参数**：

    |      | 刚度 K                  | 阻尼 D                                |
    | ---- | --------------------- | ----------------------------------- |
    | 左臂 A | {5, 5, 5, 5, 4, 3, 3} | {1.0, 1.0, 1.0, 1.0, 0.9, 0.9, 0.9} |
    | 右臂 B | {5, 5, 5, 4, 3, 3, 3} | {1.0, 1.0, 1.0, 1.0, 0.9, 0.9, 0.9} |

  - `OnSetTargetState(3)` + `OnSetImpType(1)`，双臂相同——即 **扭矩模式 + 关节阻抗 = 关节空间阻抗控制**。
- 运行循环：读反馈（DCSS 帧）→ 单位换算（度→弧度）→ 关节限位二次钳位 → 弧度转度 → `OnSetJointCmdPos_A/B(关节[7])` 下发。**只发关节目标位置，不发速度、不发力矩**。反馈只用了位置/速度（SDK 还提供电流、扭矩、温度、摩擦力估计，当前代码未用）。
- "柔性"完全由天机臂控制器**内部**实现：SDK 处于扭矩模式+关节阻抗，臂控器自己用配置的 K/D 跟踪位置目标，碰撞时臂被"压弯"而不是硬顶——这是遥操碰撞安全的保障。



### 3.6 仿真（`mujoco_sim_py/`）

MuJoCo 仿真接收**同样的关节位置指令**（`robot_sim_joint_state` 话题回灌状态），整条链路（VR 手柄 → IK → 关节指令）可以在仿真里原样调试，验证安全后再上实机。

---



## 4. 天机臂 SDK 支持的控制模式（`FxRtCSDef.h`）

**目标状态** `ArmState`：


| 值   | 模式       | 说明            |
| --- | -------- | ------------- |
| 0   | IDLE     | 下伺服           |
| 1   | POSITION | 位置跟随          |
| 2   | PVT      | PVT 轨迹        |
| 3   | TORQ     | **扭矩**（本链路使用） |
| 4   | RELEASE  | 释放            |


**阻抗类型** `m_ImpType`（扭矩模式下有效）：1 = 关节阻抗（**本链路使用**），2 = 笛卡尔阻抗，3 = 力控。

即代码实际使用：`OnSetTargetState(3)` **+** `OnSetImpType(1)` **= 关节空间阻抗控制**，之后以 100Hz 发送关节目标位置（单位：度），由臂控器内部阻抗环实现"柔性跟随"。

---



## 5. 关键问答整理（本次讨论核心）



### Q1：它实际上发的都是关节位置吗？IK 在哪做的？WBC 如何理解？

1. **是的，最终发给天机臂的就是关节位置指令。** `SlaveHardware` 的 100Hz 循环只做一件事：把上层给的目标关节角（弧度→度）通过 `OnSetJointCmdPos_A/B` 发给左右臂。不发力矩、不发速度。柔性是天机臂控制器内部用 K/D 实现的。
2. **IK 就在 RobotController 里完成**，不是独立 ROS 节点，是控制器内的一个求解器对象（`Wbc::Solve`）。
3. **WBC 名字叫 WBC，实质是"带约束的 QP 微分逆运动学"**（详见 3.4）。真正的动力学（阻抗 K/D）下沉到了天机臂控制器内部。这是遥操系统的典型分层：**上位机管"想去哪"，下位机管"怎么柔性地到那"**。



### Q2：为什么要这么设计，不直接使用天机臂控制器内部的笛卡尔阻抗控制？

五个原因：

1. **冗余自由度的解析权（最核心）**：天机每条臂 7 轴对 6 维任务，多 1 个冗余自由度。用控制器内部笛卡尔阻抗（ImpType=2），冗余怎么解是厂商黑盒：姿态会随机漂移、奇异构型附近行为不可控。自己的 QP 用零空间项驯化冗余、SVD 阻尼软化奇异、箱式约束构造性保证限位。
2. **硬件抽象**：同一套 WBC + 上层逻辑，换 URDF 就能换臂（天机/Marvin、Openarm、piper_arm、MuJoCo 仿真共用）。关节位置接口是所有臂都支持的最大公约数。
3. **仿真-实机一致性**：MuJoCo 接收的就是同样的关节位置指令，整条链路可在仿真原样调试。
4. **多模式统一在关节空间汇合**：VR 遥操（笛卡尔→IK）、主从臂（直接关节角跟随）、手柄三种模式的公共输出都是关节位置，在 `target_cmd_` 汇合走同一条下发路径。
5. **双臂是整体模型**：`tianji_dual_arms.urdf` 把左右臂建在一棵运动学树上（14 DOF 单模型），QP 在一个 12×14 雅可比上统一求解，天然支持加躯干、移动底盘等任务扩展；每条臂各自跑笛卡尔阻抗则无法做耦合任务。

**诚实的代价**：天机控制器内部跑 1kHz 级阻抗环，上位机 QP 400Hz、以太网下发 100Hz——笛卡尔刚度环路的延迟和带宽不如控制器内部笛卡尔阻抗，精密恒力接触任务会吃亏。但对遥操场景（人手本来就有延迟容忍度，安全性 > 接触精度），加上关节阻抗提供了接触柔顺兜底，这个牺牲是合理的。

### Q3：如何理解"带约束的 QP 微分逆运动学"？

按四步递进（每步对应 `wbc.cpp` 真实代码）：

1. **本质是 IK**：已知双手目标位姿求 14 个关节角。双臂 14 轴无解析解 → 选微分 IK。
2. **为什么"微分"**：正运动学在速度层面是线性的（`v = J(q)·q̇`），IK 在位置层面非线性、在速度层面线性。于是循环：PD 反馈算 v_ref → 解线性方程求 q̇ → 积分 q ← q + q̇·dt → 重算 J。注意 v_ref 来自任务空间 PD **闭环**（`kp·e − kd·v`），误差每周期重新测量，不会开环漂移。
3. **为什么"带约束"**：纯伪逆 `q̇ = J⁺v_ref` 有两个致命问题——14 未知数 12 方程解不唯一（伪逆的"范数最小"解可能让肘部甩到诡异构型）；完全无视约束（可能解出超限速的 q̇、越限位的积分结果）。QP 天生支持"在不等式约束下最小化二次代价"，一举两得。
4. **最终形态**：见 3.4 的 QP 公式。目标函数 = 任务跟踪 + 关节正则 + 姿势回归；约束 = 速度/限位箱式约束；奇异处自动阻尼。

**直观比喻**：每个周期 QP 在回答一个问题——"在不撞任何关节限位、不超过任何关节限速的前提下，14 个关节各以什么速度转，能让两只手尽量按期望速度追上 VR 手柄，顺带保持一个舒服的肘部姿势？" 当约束和任务冲突时，QP 自动降级为"可行范围内最接近"的解而不是无解崩溃——这就是遥操安全性的数学来源：**人的任意疯狂输入，经过这层 QP，出来的永远是可行域内的光滑关节运动**。

---



## 6. 关键参数速查表


| 参数              | 值                                                              | 位置                                               |
| --------------- | -------------------------------------------------------------- | ------------------------------------------------ |
| 控制线程频率          | 400 Hz                                                         | `controller.cpp`（`controlLoopThread(this, 400)`） |
| 硬件下发频率          | 100 Hz                                                         | `slave_hardware.hpp`（10ms sleep）                 |
| WBC 内部 dt       | 0.0286 s                                                       | `config/marvin_dual.yaml` `robot_config.dt`      |
| kp_pos / kp_rot | 20.0 / 15.0                                                    | `marvin_dual.yaml` `wbc.*`                       |
| kd_pos / kd_rot | kp_pos·0.3 / kp_rot·0.2                                        | `wbc.cpp`                                        |
| 任务权重 W          | 位置 10，姿态 1（12 维）                                               | `marvin_dual.yaml` `wbc.weights`                 |
| 关节正则权重          | 肩 1e-2 / 肘 1e-6 / 其余 1e-4                                      | `wbc.cpp` 构造函数                                   |
| k_null（姿势回归）    | 0.5                                                            | `Wbc.hpp`                                        |
| q_nominal       | [1.5,-1.1,-1.57,-1.5,-1.57,0,0, -1.5,-1.1,1.57,-1.5,1.57,0,0]  | `marvin_dual.yaml`                               |
| 关节速度上限 v_max    | 3.1416 rad/s（全关节）                                              | `marvin_dual.yaml`                               |
| 奇异阈值 ε / 阻尼 λ   | 0.05 / 1e-4 ~ 1e-2                                             | `wbc.cpp`                                        |
| motion_scale    | 1.2                                                            | `marvin_dual.yaml` `teleop.motion_scale`         |
| vr_to_robot     | [1,0,0, 0,0,1, 0,-1,0]                                         | `marvin_dual.yaml`                               |
| VR 话题           | `pico/hand_poses`（`teleop.vr_topic` 可配）                        | `marvin_dual.yaml`                               |
| 天机臂 IP          | 192.168.110.24                                                 | `slave_hardware.hpp`                             |
| SDK 模式          | TargetState=3（扭矩）+ ImpType=1（关节阻抗）                             | `slave_hardware.hpp` / `FxRtCSDef.h`             |
| 阻抗 K（A/B）       | A:{5,5,5,5,4,3,3} / B:{5,5,5,4,3,3,3}                          | `slave_hardware.hpp`                             |
| 阻抗 D（A/B）       | {1.0,1.0,1.0,1.0,0.9,0.9,0.9}                                  | `slave_hardware.hpp`                             |
| 夹爪              | PGC ×2，串口 `/dev/gripper_l_usb`、`/dev/gripper_r_usb`，握把键 toggle | `controller.cpp`                                 |


---



## 7. 关键文件索引（本地克隆路径）

- 输入侧：`roboteleop/src/pico_teleop_pkg/pico_teleop_pkg/` 下 `pico_teleop_sdk_node.py`（XRoboToolkit SDK）、`pico_teleop_node.py`（gRPC PXREA）、`pico_connection_quality.py`
- 控制器：`robot_teleop/robot_teleop/controller/src/controller.cpp`（400Hz 主循环 + FSM）
- 位姿映射：`robot_teleop/robot_teleop/controller/include/teleoperator/vr_teleoperator.hpp`
- QP 求解器：`robot_teleop/robot_teleop/controller/src/wbc/wbc.cpp`（+ `id_wbc.cpp` 逆动力学版，未启用）
- 硬件层：`robot_teleop/robot_teleop/controller/include/hardware/slave_hardware.hpp`（+ `leader_hardware.hpp`、`openarm_hardware.hpp`）
- SDK：`robot_teleop/robot_teleop/controller/third_party/marvin_sdk/`（`controlSDK/FxRtCSDef.h` 定义模式枚举）
- 模型：`robot_teleop/robot_teleop/robot_example/tj_arm_description/urdf/TJ_arm/tianji_dual_arms.urdf`
- 配置：`robot_teleop/robot_teleop/controller/config/marvin_dual.yaml`
- 仿真：`robot_teleop/robot_teleop/mujoco_sim_py/`（configs/marvin_dual.yaml）

---



## 8. 架构总结：三层各司其职

```
VR 手柄 ──► PD（任务空间）──► 带约束 QP 微分 IK ──► 关节位置 ──► 天机臂关节阻抗
           "追得上"             "可行"              "柔软"
```

- **PD 管"追得上"**：末端误差 → 期望速度（闭环）；
- **QP 管"可行"**：期望速度 → 满足限位/限速/奇异约束的最优关节运动；
- **阻抗管"柔软"**：关节位置目标 → 天机臂内部 K/D 阻抗环柔顺跟踪，碰撞不硬顶。

这套分层的设计哲学是：**把"冗余解析 + 奇异性处理 + 限位安全"这些最关键的运动学控制权留在自己手里，把柔顺执行交给厂商的阻抗环，换来可控性、可移植性、可仿真性和多臂多模式统一架构**——代价只是几十毫秒的环路延迟，对遥操完全可接受。