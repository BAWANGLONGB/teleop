# pico teleop

## 迁移

### Wave 1：PICO → MuJoCo

  先迁移：

  - XR native binding；
  - xr_client；
  - 坐标映射；
  - controller；
  - Marvin IK；
  - MuJoCo adapter；
  - 仿真 CLI。

  验收：

  .venv/bin/python -m unittest \
    tests.test_marvin_controller \
    tests.test_marvin_simulation -v

  这是第一条完整闭环，不连接机器人。

  ### Wave 2：Marvin 与 DAS

  迁移厂家适配器和实机入口，保持这些行为不变：

  - 内部关节单位为弧度，厂家边界才转换成角度；
  - 14 轴顺序固定为 [A1..A7, B1..B7]；
  - XR 断流时保持当前目标；
  - Grip 松开时重新锚定；
  - B 回位只在双 Grip 松开时生效；
  - 实机确认参数不能删除。

  验收：

  .venv/bin/python -m unittest \
    tests.test_marvin_interfaces \
    tests.test_marvin_entrypoints -v

  ### Wave 3：ROS 与数据采集

  迁移：

  - ROS 协议；
  - PICO/DAS 发布器；
  - recorder；
  - Episode 后处理；
  - AV1/H.264/MJPEG；
  - supervisor。

  ROS topic 和已有 MCAP 格式保持不变。

  验收：

  .venv/bin/python -m unittest \
    tests.test_marvin_ros_data \
    tests.test_collection_config \
    tests.test_episode_postprocessor \
    tests.test_episode_video -v

  ### Wave 4：UI

  最后迁移 UI 后端，让它只负责：

  - 启动/停止 CLI；
  - 查询设备状态；
  - 展示日志和预览；
  - 管理数据集。

  UI 不直接导入控制器或厂家 SDK。

  验收：

  .venv/bin/python -m xr_marvin_teleop.web.server --self-test
  node ui/test.mjs
