"""V2 wire contract. ROS imports stay lazy so offline configuration needs no ROS."""

from collections import OrderedDict
import json
import math
import time

VERSION = 2
ARM_NAMES = ("left", "right")
JOINT_NAMES = tuple(f"Joint{i}_{side}" for side in ("L", "R") for i in range(1, 8))
GRIPPER_NAMES = ("left_gripper_width", "right_gripper_width")
PICO_TOPIC_PREFIX = "/raw/pico"
PICO_POSES_TOPIC = f"{PICO_TOPIC_PREFIX}/poses"
PICO_JOY_TOPIC = f"{PICO_TOPIC_PREFIX}/joy"
PICO_STATUS_TOPIC = f"{PICO_TOPIC_PREFIX}/status"
PICO_TOPICS = (PICO_POSES_TOPIC, PICO_JOY_TOPIC)
MARVIN_JOINT_STATE_TOPIC = "/raw/marvin/joint_state"
MARVIN_JOINT_COMMAND_TOPIC = "/command/marvin/joint_target"
DAS_COMMAND_TOPIC = "/command/das/target"
DAS_STATE_TOPICS = tuple(f"/raw/das/{side}/state" for side in ARM_NAMES)
DAS_TACTILE_TOPICS = tuple(f"/raw/das/{side}/tactile" for side in ARM_NAMES)
DAS_IMAGE_TOPICS = tuple(f"/raw/das/{side}/image" for side in ARM_NAMES)
DAS_COMPRESSED_IMAGE_TOPICS = tuple(f"{topic}/compressed" for topic in DAS_IMAGE_TOPICS)
DAS_COMPRESSED_IMAGE_STATUS_TOPICS = tuple(
    f"{topic}/status" for topic in DAS_COMPRESSED_IMAGE_TOPICS
)
MARVIN_TCP_STATE_TOPICS = tuple(
    f"/raw/marvin/{side}/tcp_pose" for side in ARM_NAMES
)
MARVIN_TCP_COMMAND_TOPICS = tuple(
    f"/command/marvin/{side}/tcp_target" for side in ARM_NAMES
)
DIAGNOSTICS_TOPIC = "/diagnostics"
PICO_AXES = ("left_grip", "right_grip", "left_trigger", "right_trigger", "left_thumbstick_y", "right_thumbstick_y")
PICO_BUTTONS = ("a", "b", "x", "y")


def stamp(header, timestamp_ns, frame_id=""):
    header.stamp.sec, header.stamp.nanosec = divmod(int(timestamp_ns), 1_000_000_000)
    header.frame_id = frame_id


def stamp_ns(message):
    value = message.header.stamp if hasattr(message, "header") else message.timestamp
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def status_topic(topic):
    return PICO_STATUS_TOPIC if topic in PICO_TOPICS else topic + "/status"


def sample_status(topics, timestamp_ns, session, sequence, steady_ns, *, valid=True,
                  source_timestamp_ns=0, source_clock="unavailable", command=False, **extra):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    message = DiagnosticArray()
    stamp(message.header, timestamp_ns)
    values = dict(protocol_version=VERSION, topics=list(topics), publisher_session_id=session,
                  sequence_id=int(sequence), valid=bool(valid),
                  source_timestamp_ns=int(source_timestamp_ns), source_clock=source_clock,
                  **{("issue_steady_ns" if command else "receive_steady_ns"): int(steady_ns)}, **extra)
    status = DiagnosticStatus(name="teleop_sample", hardware_id=session,
                              level=DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR,
                              message="valid" if valid else "invalid")
    # Fixed ROS arrays deserialize to NumPy scalars; preserve their numeric values.
    status.values = [KeyValue(key=key, value=json.dumps(value, separators=(",", ":"), default=lambda v: v.item()))
                     for key, value in values.items()]
    message.status = [status]
    return message


def status_values(message):
    if len(message.status) != 1 or message.status[0].name != "teleop_sample":
        raise ValueError("expected one teleop sample diagnostic")
    entries = message.status[0].values
    values = {item.key: json.loads(item.value) for item in entries}
    if len(values) != len(entries) or values.get("protocol_version") != VERSION:
        raise ValueError("invalid sample metadata version/keys")
    if (not isinstance(values.get("publisher_session_id"), str)
            or not values["publisher_session_id"]
            or type(values.get("sequence_id")) is not int or values["sequence_id"] <= 0
            or type(values.get("valid")) is not bool
            or not isinstance(values.get("topics"), list)
            or not values["topics"] or not all(isinstance(t, str) and t.startswith("/") for t in values["topics"])
            or len(set(values["topics"])) != len(values["topics"])
            or type(values.get("source_timestamp_ns")) is not int or values["source_timestamp_ns"] < 0):
        raise ValueError("invalid sample identity")
    return values


def ordered_values(names, values, expected):
    if len(names) != len(values) or len(set(names)) != len(names) or set(names) != set(expected):
        raise ValueError("joint names must match the contract exactly")
    mapping = dict(zip(names, values))
    result = [float(mapping[name]) for name in expected]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("joint values must be finite")
    return result


def joint_positions(message, expected=JOINT_NAMES):
    if hasattr(message, "points"):
        if len(message.points) != 1:
            raise ValueError("the SDK gateway accepts exactly one immediate target")
        point = message.points[0]
        if point.time_from_start.sec or point.time_from_start.nanosec:
            raise ValueError("scheduled trajectories are not supported by the SDK gateway")
        if point.velocities or point.accelerations or point.effort:
            raise ValueError("the SDK gateway accepts position targets only")
        return ordered_values(message.joint_names, point.positions, expected)
    return ordered_values(message.name, message.position, expected)


def trajectory(values, names, timestamp_ns):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    message = JointTrajectory(joint_names=list(names))
    stamp(message.header, timestamp_ns)
    message.points = [JointTrajectoryPoint(positions=ordered_values(names, values, names))]
    return message


def gripper_targets(message, configurations):
    from ..adapters.das_finger import closedness_to_das_distances
    values = joint_positions(message, GRIPPER_NAMES)
    if len(configurations) != 2:
        raise ValueError("two gripper calibrations are required")
    if any(not 0.0 <= value <= 1.0 for value in values):
        raise ValueError("gripper openness must be within [0, 1]")
    return closedness_to_das_distances([1.0 - value for value in values], configurations)


def joint_state(values, names, timestamp_ns, velocities=()):
    from sensor_msgs.msg import JointState
    message = JointState(name=list(names), position=[float(v) for v in values],
                         velocity=[float(v) for v in velocities])
    stamp(message.header, timestamp_ns)
    return message


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def pico_messages(snapshot, timestamp_ns):
    from geometry_msgs.msg import Pose, PoseArray
    from sensor_msgs.msg import Joy
    poses, joy = PoseArray(), Joy()
    for message in (poses, joy):
        stamp(message.header, timestamp_ns, "openxr_local")
    if snapshot is not None:
        # Keep the two controller indices stable; Head is the optional third pose.
        values_list = [snapshot.left_controller_pose, snapshot.right_controller_pose]
        if snapshot.head_pose is not None:
            values_list.append(snapshot.head_pose)
        for values in values_list:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, values[:3])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, values[3:])
            poses.poses.append(pose)
        joy.axes = list(map(float, (*snapshot.grip_values, *snapshot.trigger_values, *snapshot.thumbstick_y_values)))
        joy.buttons = [int(getattr(snapshot, "button_" + name)) for name in PICO_BUTTONS]
    return poses, joy


def tactile_grid(payload, side, timestamp_ns):
    from foxglove_msgs.msg import Grid, PackedElementField
    if len(payload) != 448:
        raise ValueError("DAS tactile packet must contain exactly 448 bytes")
    message = Grid()
    message.timestamp.sec, message.timestamp.nanosec = divmod(int(timestamp_ns), 1_000_000_000)
    message.frame_id = f"das_{side}_tactile_index"
    message.pose.orientation.w = 1.0
    message.column_count, message.row_stride, message.cell_stride = 224, 224, 1
    message.cell_size.x = message.cell_size.y = 1.0
    message.fields = [PackedElementField(name="raw_value", offset=0, type=PackedElementField.UINT8)]
    message.data = bytes(payload)
    return message


class SampleJoiner:
    """Bounded exact-stamp join with session/sequence and local freshness checks."""

    def __init__(self, topics, max_age_ns):
        self.topics = tuple(topics)
        self.max_age_ns = int(max_age_ns)
        self.pending = OrderedDict()
        self.session = None
        self.retired_sessions = set()
        self.sequence = 0
        self.last_outcome = "waiting"
        self.expired_pending = 0
        self.evicted_pending = 0

    def push(self, topic, message):
        self.last_outcome = "malformed"
        now, wall = time.monotonic_ns(), time.time_ns()
        timestamp = stamp_ns(message)
        if timestamp <= 0 or abs(wall - timestamp) > self.max_age_ns:
            self.last_outcome = "stale_stamp"
            return None
        for key in list(self.pending):
            if now - self.pending[key][0] > self.max_age_ns:
                del self.pending[key]
                self.expired_pending += 1
        entry = self.pending.setdefault(timestamp, (now, {}))
        entry[1][topic] = message
        while len(self.pending) > 32:
            self.pending.popitem(last=False)
            self.evicted_pending += 1
        parts = entry[1]
        if "status" not in parts:
            self.last_outcome = "waiting_status"
            return None
        values = status_values(parts["status"])
        if set(values["topics"]) != set(self.topics):
            raise ValueError("sample metadata targets unexpected topics")
        session, sequence = values["publisher_session_id"], values["sequence_id"]
        if session in self.retired_sessions or (session == self.session and sequence <= self.sequence):
            self.last_outcome = "old_sequence_or_session"
            self.pending.pop(timestamp, None)
            return None
        if values["valid"] and any(name not in parts for name in self.topics):
            self.last_outcome = "waiting_payload"
            return None
        if session != self.session:
            if self.session is not None:
                self.retired_sessions.add(self.session)
            self.session, self.sequence = session, 0
            self.pending.clear()
        self.sequence = sequence
        self.pending.pop(timestamp, None)
        self.last_outcome = "joined" if values["valid"] else "source_invalid"
        return parts, values, entry[0]
