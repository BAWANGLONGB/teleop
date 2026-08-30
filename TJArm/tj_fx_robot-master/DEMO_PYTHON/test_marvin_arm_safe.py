#!/usr/bin/env python3
"""Marvin 双臂安全验收脚本。

默认只读：检查 SDK ABI、连接、A/B 双臂帧号、状态、关节位置/速度和故障码，
不切换模式、不清错、不下发运动目标。

只有同时使用 ``--motion-test --enable-hardware`` 并在终端输入确认语句时，
才会以当前反馈为起点，让单臂的单个关节小角度往返。

示例：
    # 只读检查 10 秒
    python DEMO_PYTHON/test_marvin_arm_safe.py --duration 10

    # A 臂 7 号关节 +0.5° 再返回（会真实运动）
    python DEMO_PYTHON/test_marvin_arm_safe.py --motion-test --enable-hardware \
        --arm A --joint 7 --delta-deg 0.5
"""

from __future__ import annotations

import argparse
import csv
import ipaddress
import math
import os
import platform
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
SDK_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SDK_ROOT))

from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot  # noqa: E402


ARM_INDEX = {"A": 0, "B": 1}
STATE_NAMES = {
    0: "IDLE/下使能",
    1: "POSITION/位置",
    2: "PVT",
    3: "TORQUE/阻抗",
    4: "RELEASE/协作释放",
    100: "ERROR/故障",
}
CONFIRMATION = "ESTOP_READY_CLEAR_WORKSPACE"
SERIAL_MODULUS = 1_000_000


class TestFailure(RuntimeError):
    """用于可预期的验收失败。"""


@dataclass
class SerialWatch:
    last: int | None = None
    changes: int = 0
    last_change_time: float | None = None

    def update(self, serial: int, now: float) -> None:
        if serial <= 0:
            return
        if self.last is None:
            self.last = serial
            self.last_change_time = now
            return
        if serial != self.last:
            self.changes += 1
            self.last = serial
            self.last_change_time = now

    def age(self, now: float) -> float:
        if self.last_change_time is None:
            return math.inf
        return now - self.last_change_time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Marvin 机械臂默认只读检查，可选单关节小角度往返测试。"
    )
    parser.add_argument("--ip", default="192.168.1.190", help="控制器 IP（默认：%(default)s）")
    parser.add_argument("--duration", type=float, default=10.0, help="只读采样时长/秒（默认：%(default)s）")
    parser.add_argument("--rate", type=float, default=20.0, help="只读采样频率/Hz（默认：%(default)s）")
    parser.add_argument("--feedback-timeout", type=float, default=0.5, help="帧号停更超时/秒")
    parser.add_argument("--csv", type=Path, help="可选：保存反馈 CSV 的路径")

    motion = parser.add_argument_group("实机微动（危险，默认关闭）")
    motion.add_argument("--motion-test", action="store_true", help="启用单关节小角度往返测试")
    motion.add_argument("--enable-hardware", action="store_true", help="显式授权实机运动")
    motion.add_argument("--arm", choices=("A", "B"), default="A", help="微动手臂（默认：%(default)s）")
    motion.add_argument("--joint", type=int, choices=range(1, 8), default=7, metavar="1..7", help="关节号")
    motion.add_argument("--delta-deg", type=float, default=0.5, help="相对角度，绝对值不得超过 1°")
    motion.add_argument("--move-seconds", type=float, default=2.0, help="单程轨迹时长/秒")
    motion.add_argument("--hold-seconds", type=float, default=0.5, help="终点停留时长/秒")
    motion.add_argument("--vel-percent", type=int, default=5, help="供应商速度限制百分比，最大 10")
    motion.add_argument("--acc-percent", type=int, default=5, help="供应商加速度限制百分比，最大 10")
    motion.add_argument("--motion-rate", type=float, default=50.0, help="微动指令频率/Hz（默认：%(default)s）")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    try:
        address = ipaddress.ip_address(args.ip)
    except ValueError as exc:
        raise TestFailure(f"无效 IP：{args.ip}") from exc
    if address.version != 4:
        raise TestFailure("Marvin SDK 测试脚本只接受 IPv4")
    if args.duration < 2.0:
        raise TestFailure("--duration 不得小于 2 秒")
    if not 1.0 <= args.rate <= 100.0:
        raise TestFailure("--rate 必须在 1..100 Hz")
    if not 0.1 <= args.feedback_timeout <= 2.0:
        raise TestFailure("--feedback-timeout 必须在 0.1..2.0 秒")
    if args.motion_test and not args.enable_hardware:
        raise TestFailure("微动测试还必须显式添加 --enable-hardware")
    if args.enable_hardware and not args.motion_test:
        raise TestFailure("--enable-hardware 只能与 --motion-test 同时使用")
    if not math.isfinite(args.delta_deg) or not 0.2 <= abs(args.delta_deg) <= 1.0:
        raise TestFailure("--delta-deg 绝对值必须在 0.2..1.0°")
    if not 1.0 <= args.move_seconds <= 10.0:
        raise TestFailure("--move-seconds 必须在 1..10 秒")
    if not 0.0 <= args.hold_seconds <= 5.0:
        raise TestFailure("--hold-seconds 必须在 0..5 秒")
    if not 1 <= args.vel_percent <= 10 or not 1 <= args.acc_percent <= 10:
        raise TestFailure("首次微动的速度和加速度百分比必须在 1..10")
    if not 10.0 <= args.motion_rate <= 50.0:
        raise TestFailure("--motion-rate 必须在 10..50 Hz")


def all_finite(values: Iterable[Any]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def require_ok(value: Any, operation: str) -> None:
    if not value:
        raise TestFailure(f"SDK 操作失败：{operation}")


def state_label(state: int) -> str:
    return STATE_NAMES.get(state, f"UNKNOWN({state})")


def subscribe_checked(robot: Marvin_Robot, dcss: DCSS) -> dict[str, Any]:
    data = robot.subscribe(dcss)
    if not isinstance(data, dict) or len(data.get("states", [])) != 2 or len(data.get("outputs", [])) != 2:
        raise TestFailure("SDK 返回的 DCSS 数据结构异常")
    for arm_index, arm in enumerate(("A", "B")):
        output = data["outputs"][arm_index]
        if not all_finite(output.get("fb_joint_pos", [])) or len(output.get("fb_joint_pos", [])) != 7:
            raise TestFailure(f"{arm} 臂关节位置非法")
        if not all_finite(output.get("fb_joint_vel", [])) or len(output.get("fb_joint_vel", [])) != 7:
            raise TestFailure(f"{arm} 臂关节速度非法")
    return data


class CsvRecorder:
    HEADER = [
        "monotonic_s",
        "wall_time",
        "arm",
        "frame_serial",
        "state",
        "error_code",
        *[f"q{i}_deg" for i in range(1, 8)],
        *[f"dq{i}_deg_s" for i in range(1, 8)],
    ]

    def __init__(self, path: Path | None):
        self._file = None
        self._writer = None
        if path is not None:
            path = path.expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(self.HEADER)
            print(f"CSV：{path}")

    def write(self, now: float, data: dict[str, Any]) -> None:
        if self._writer is None:
            return
        wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        for index, arm in enumerate(("A", "B")):
            state = data["states"][index]
            output = data["outputs"][index]
            self._writer.writerow(
                [
                    f"{now:.6f}",
                    wall,
                    arm,
                    output["frame_serial"],
                    state["cur_state"],
                    state["err_code"],
                    *output["fb_joint_pos"],
                    *output["fb_joint_vel"],
                ]
            )

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()


def inspect_feedback(
    robot: Marvin_Robot,
    dcss: DCSS,
    duration: float,
    rate: float,
    timeout: float,
    recorder: CsvRecorder,
) -> tuple[dict[str, Any], dict[str, SerialWatch]]:
    watches = {"A": SerialWatch(), "B": SerialWatch()}
    deadline = time.monotonic() + duration
    period = 1.0 / rate
    next_print = 0.0
    last_data: dict[str, Any] | None = None

    while time.monotonic() < deadline:
        cycle_start = time.monotonic()
        data = subscribe_checked(robot, dcss)
        last_data = data
        recorder.write(cycle_start, data)

        for index, arm in enumerate(("A", "B")):
            serial = int(data["outputs"][index]["frame_serial"])
            watches[arm].update(serial, cycle_start)
            if watches[arm].last is not None and watches[arm].age(cycle_start) > timeout:
                raise TestFailure(f"{arm} 臂 frame_serial 已停更 {watches[arm].age(cycle_start):.3f} 秒")

        if cycle_start >= next_print:
            for index, arm in enumerate(("A", "B")):
                state = data["states"][index]
                output = data["outputs"][index]
                q_text = ", ".join(f"{q:8.3f}" for q in output["fb_joint_pos"])
                print(
                    f"{arm} frame={output['frame_serial']:7d}  "
                    f"state={state_label(state['cur_state']):<18} err={state['err_code']}  q_deg=[{q_text}]"
                )
            next_print = cycle_start + 1.0

        remaining = period - (time.monotonic() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    if last_data is None:
        raise TestFailure("未采集到反馈")
    for arm, watch in watches.items():
        if watch.last is None or watch.changes < 2:
            raise TestFailure(f"{arm} 臂 frame_serial 未非零持续递增")
    return last_data, watches


def validate_motion_preconditions(data: dict[str, Any], servo_faults: dict[str, str]) -> None:
    for index, arm in enumerate(("A", "B")):
        state = data["states"][index]
        output = data["outputs"][index]
        if int(state["cur_state"]) != 0:
            raise TestFailure(f"微动前 {arm} 臂必须处于 IDLE/下使能，当前为 {state_label(state['cur_state'])}")
        if int(state["err_code"]) != 0:
            raise TestFailure(f"{arm} 臂存在故障 err={state['err_code']}；脚本不会自动清错")
        if servo_faults.get(arm) != "None":
            raise TestFailure(f"{arm} 臂存在伺服故障：{servo_faults.get(arm)}；脚本不会自动清错")
        if max(abs(float(value)) for value in output["fb_joint_vel"]) > 0.5:
            raise TestFailure(f"{arm} 臂尚未静止（关节速度 > 0.5°/s）")


def confirm_motion(args: argparse.Namespace, data: dict[str, Any], servo_faults: dict[str, str]) -> None:
    validate_motion_preconditions(data, servo_faults)
    if not sys.stdin.isatty():
        raise TestFailure("微动测试必须在交互式终端运行")

    print("\n即将实机运动：")
    print(f"  机器人 IP : {args.ip}")
    print(f"  目标        : {args.arm} 臂 J{args.joint} {args.delta_deg:+.3f}°，然后返回起点")
    print(f"  速度/加速度 : {args.vel_percent}% / {args.acc_percent}%")
    print("  请确认物理急停有效、有人值守急停、工作区无人且无障碍物。")
    entered = input(f"\n请完整输入 {CONFIRMATION} 继续：").strip()
    if entered != CONFIRMATION:
        raise TestFailure("确认语句不匹配，已取消微动测试")


def send_position_setup(robot: Marvin_Robot, arm: str, current_q: list[float], vel: int, acc: int) -> None:
    require_ok(robot.clear_set(), "clear_set")
    require_ok(robot.set_vel_acc(arm=arm, velRatio=vel, AccRatio=acc), "set_vel_acc")
    require_ok(robot.set_joint_cmd_pose(arm=arm, joints=current_q), "设置当前位置为初始目标")
    require_ok(robot.set_state(arm=arm, state=1), "切换 POSITION 模式")
    require_ok(robot.send_cmd(), "send_cmd(POSITION setup)")


def send_joint_target(robot: Marvin_Robot, arm: str, target: list[float]) -> None:
    require_ok(robot.clear_set(), "clear_set")
    require_ok(robot.set_joint_cmd_pose(arm=arm, joints=target), "set_joint_cmd_pose")
    require_ok(robot.send_cmd(), "send_cmd(joint target)")


def disable_arm(robot: Marvin_Robot, arm: str) -> None:
    require_ok(robot.clear_set(), "clear_set")
    require_ok(robot.set_state(arm=arm, state=0), f"{arm} 臂下使能")
    require_ok(robot.send_cmd(), "send_cmd(IDLE)")


def wait_for_state(robot: Marvin_Robot, dcss: DCSS, arm: str, expected: int, timeout: float = 2.0) -> dict[str, Any]:
    index = ARM_INDEX[arm]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = subscribe_checked(robot, dcss)
        state = data["states"][index]
        if int(state["err_code"]) != 0 or int(state["cur_state"]) == 100:
            raise TestFailure(f"{arm} 臂进入故障：state={state['cur_state']}, err={state['err_code']}")
        if int(state["cur_state"]) == expected:
            return data
        time.sleep(0.02)
    raise TestFailure(f"{arm} 臂未在 {timeout:.1f}s 内进入 {state_label(expected)}")


def wait_for_joint_target(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    joint_index: int,
    target_deg: float,
    tolerance_deg: float,
    timeout: float = 3.0,
) -> dict[str, Any]:
    index = ARM_INDEX[arm]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = subscribe_checked(robot, dcss)
        state = data["states"][index]
        if int(state["cur_state"]) != 1 or int(state["err_code"]) != 0:
            raise TestFailure(f"{arm} 臂等待到位时状态异常：state={state['cur_state']}, err={state['err_code']}")
        actual = float(data["outputs"][index]["fb_joint_pos"][joint_index])
        if abs(actual - target_deg) <= tolerance_deg:
            return data
        time.sleep(0.02)
    raise TestFailure(
        f"{arm} 臂 J{joint_index + 1} 未在 {timeout:.1f}s 内到位，"
        f"目标={target_deg:.3f}°，容差={tolerance_deg:.3f}°"
    )


def run_segment(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    start: list[float],
    end: list[float],
    seconds: float,
    rate: float,
    feedback_timeout: float,
) -> dict[str, Any]:
    index = ARM_INDEX[arm]
    steps = max(2, int(math.ceil(seconds * rate)))
    period = 1.0 / rate
    watch = SerialWatch()
    last_data: dict[str, Any] | None = None

    for step in range(1, steps + 1):
        cycle_start = time.monotonic()
        # 余弦插值使起止速度平滑，不直接向控制器丢阶跃。
        phase = step / steps
        blend = 0.5 - 0.5 * math.cos(math.pi * phase)
        target = [a + blend * (b - a) for a, b in zip(start, end)]
        send_joint_target(robot, arm, target)

        data = subscribe_checked(robot, dcss)
        last_data = data
        state = data["states"][index]
        output = data["outputs"][index]
        serial = int(output["frame_serial"])
        watch.update(serial, cycle_start)
        if int(state["cur_state"]) != 1 or int(state["err_code"]) != 0:
            raise TestFailure(f"{arm} 臂微动中状态异常：state={state['cur_state']}, err={state['err_code']}")
        if watch.last is not None and watch.age(cycle_start) > feedback_timeout:
            raise TestFailure(f"{arm} 臂微动中反馈停更")
        if max(abs(float(v)) for v in output["fb_joint_vel"]) > 5.0:
            raise TestFailure(f"{arm} 臂微动中关节速度超过 5°/s")

        remaining = period - (time.monotonic() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    if last_data is None or watch.changes < 2:
        raise TestFailure(f"{arm} 臂微动期间反馈未持续更新")
    return last_data


def motion_test(
    robot: Marvin_Robot,
    dcss: DCSS,
    args: argparse.Namespace,
    servo_faults: dict[str, str],
) -> None:
    arm = args.arm
    index = ARM_INDEX[arm]
    joint_index = args.joint - 1
    # 操作员输入确认语句后再做一次实时检查，不复用确认前的快照。
    initial_data = subscribe_checked(robot, dcss)
    validate_motion_preconditions(initial_data, servo_faults)
    start_q = [float(q) for q in initial_data["outputs"][index]["fb_joint_pos"]]
    target_q = start_q.copy()
    target_q[joint_index] += args.delta_deg
    armed = False

    print(f"\n初始 {arm} 臂关节角（deg）：{start_q}")
    try:
        armed = True
        send_position_setup(robot, arm, start_q, args.vel_percent, args.acc_percent)
        wait_for_state(robot, dcss, arm, expected=1)
        print(f"微动至 J{args.joint}={target_q[joint_index]:.3f}° ...")
        run_segment(robot, dcss, arm, start_q, target_q, args.move_seconds, args.motion_rate, args.feedback_timeout)
        time.sleep(args.hold_seconds)
        outward_tolerance = max(0.08, abs(args.delta_deg) * 0.25)
        wait_for_joint_target(robot, dcss, arm, joint_index, target_q[joint_index], outward_tolerance)
        print("返回测试起点 ...")
        run_segment(
            robot, dcss, arm, target_q, start_q, args.move_seconds, args.motion_rate, args.feedback_timeout
        )
        final_data = wait_for_joint_target(robot, dcss, arm, joint_index, start_q[joint_index], 0.15)
        final_q = float(final_data["outputs"][index]["fb_joint_pos"][joint_index])
        error = abs(final_q - start_q[joint_index])
        print(f"返回误差：{error:.3f}°")
        if error > 1.0:
            raise TestFailure(f"返回起点误差 {error:.3f}° > 1.0°")
    except BaseException:
        if armed:
            try:
                robot.soft_stop(arm)
            except Exception as exc:  # pragma: no cover - 只在 SDK 异常退出时执行
                print(f"警告：软停失败：{exc}", file=sys.stderr)
        raise
    finally:
        if armed:
            try:
                disable_arm(robot, arm)
                print(f"{arm} 臂已下使能。")
            except Exception as exc:
                print(f"严重警告：{arm} 臂下使能请求失败：{exc}", file=sys.stderr)


def install_signal_handlers() -> None:
    def stop_handler(signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except TestFailure as exc:
        print(f"参数错误：{exc}", file=sys.stderr)
        return 2
    install_signal_handlers()
    recorder: CsvRecorder | None = None
    robot: Marvin_Robot | None = None
    connected = False

    print(f"主机：{platform.platform()} / {platform.machine()}")
    print(f"SDK 路径：{SDK_ROOT / 'SDK_PYTHON'}")
    mode_name = "实机微动" if args.motion_test else "只读检查"
    print(f"模式：{mode_name}")

    try:
        recorder = CsvRecorder(args.csv)
        robot = Marvin_Robot()
        compat_ret, byte_order = robot.check_sdk_type_compat()
        if compat_ret < 0:
            raise TestFailure(f"SDK ABI 兼容性检查失败，mask=0x{-compat_ret:x}")
        print(f"SDK ABI：通过，{'little-endian' if byte_order == 0 else 'big-endian'}")
        print(f"SDK 版本：{robot.SDK_version()}")

        if not robot.connect(args.ip):
            raise TestFailure("连接失败：请检查 IP、网络、防火墙，以及 MarvinPlatform/其他 SDK 进程是否占用")
        connected = True
        print(f"已连接 {args.ip}，开始双臂反馈验收 ...")

        dcss = DCSS()
        data, watches = inspect_feedback(
            robot, dcss, args.duration, args.rate, args.feedback_timeout, recorder
        )
        servo_faults: dict[str, str] = {}
        for index, arm in enumerate(("A", "B")):
            state = data["states"][index]
            servo_faults[arm] = robot.get_servo_error_code(arm, lang="CN")
            print(
                f"{arm} 臂验收通过：帧变化 {watches[arm].changes} 次，"
                f"state={state_label(state['cur_state'])}，err={state['err_code']}，"
                f"servo={servo_faults[arm]}"
            )

        if args.motion_test:
            confirm_motion(args, data, servo_faults)
            motion_test(robot, dcss, args, servo_faults)
            print("实机微动测试通过。")
        else:
            print("只读测试通过；未切换模式、未清错、未下发运动指令。")
        return 0
    except KeyboardInterrupt:
        print("\n测试被中止。", file=sys.stderr)
        return 130
    except TestFailure as exc:
        print(f"\n测试失败：{exc}", file=sys.stderr)
        return 2
    except (OSError, ValueError) as exc:
        print(f"\n本地 SDK 或文件错误：{exc}", file=sys.stderr)
        return 3
    finally:
        if recorder is not None:
            recorder.close()
        if robot is not None:
            if connected:
                try:
                    robot.release_robot()
                    print("SDK 连接已释放。")
                except Exception as exc:
                    print(f"警告：release_robot() 失败：{exc}", file=sys.stderr)
            else:
                # SDK 对未建立连接的 OnRelease 也安全，用于释放本地库资源。
                try:
                    robot.release_robot()
                except Exception:
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
