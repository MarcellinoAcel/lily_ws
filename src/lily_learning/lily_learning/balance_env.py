# ─────────────────────────────────────────────────────────────────────────────
# envs/balance_env.py  —  Gymnasium environment wrapping ROS2 + Gazebo
# ─────────────────────────────────────────────────────────────────────────────

import time
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lily_learning.config import TOPICS, SERVICES, STAND_POSE, JOINT_NAMES
from lily_learning.config import OBS_LOW, OBS_HIGH, ACT_LOW, ACT_HIGH
from lily_learning.config import MAX_EPISODE_STEPS, STEP_DURATION_SEC, STAND_SETTLE_SEC, FALL_PITCH_THRESH
from lily_learning.config import REWARD

# ── ROS2 Node (handles all comms with Gazebo) ─────────────────────────────────
class RobotBridge(Node):
    def __init__(self, node_name="balance_env_node"):
        super().__init__(node_name)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, TOPICS["cmd_vel"], 10)

        # Subscribers
        self.create_subscription(Imu,        TOPICS["imu"],          self._imu_cb,   10)
        self.create_subscription(JointState, TOPICS["joint_states"],  self._js_cb,    10)

        # Action client
        self.action_client = ActionClient(self, FollowJointTrajectory, TOPICS["action_server"])
        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server()

        # Service clients
        self.reset_client = self.create_client(Empty, SERVICES["reset"])

        # State
        self.pitch          = 0.0
        self.pitch_velocity = 0.0
        self.joint_positions = {name: STAND_POSE[name] for name in JOINT_NAMES}
        self.imu_ready      = False
        self.js_ready       = False

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        # Quaternion → pitch (rotation around Y)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.pitch_velocity = msg.angular_velocity.y
        self.imu_ready = True

    def _js_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]
        self.js_ready = True

    # ── Observation ───────────────────────────────────────────────────────────
    def get_observation(self) -> np.ndarray:
        return np.array([
            self.pitch,
            self.pitch_velocity,
            self.joint_positions["left_thigh_joint"],
            self.joint_positions["left_knee_joint"],
            self.joint_positions["right_thigh_joint"],
            self.joint_positions["right_knee_joint"],
        ], dtype=np.float32)

    # ── Send joint targets ────────────────────────────────────────────────────
    def send_joints(self, targets: dict, duration_sec: float = 0.1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        positions = [targets.get(n, STAND_POSE[n]) for n in JOINT_NAMES]
        point = JointTrajectoryPoint()
        point.positions = positions

        # Convert float seconds to builtin_interfaces Duration
        sec  = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [point]
        self.action_client.send_goal_async(goal)   # fire and forget

    # ── Send wheel speed ──────────────────────────────────────────────────────
    def send_cmd_vel(self, linear_x: float, angular_z: float = 0.0):
        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    # ── Stop robot ────────────────────────────────────────────────────────────
    def stop(self):
        self.send_cmd_vel(0.0)

    # ── Reset Gazebo simulation ───────────────────────────────────────────────
    def reset_simulation(self):
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Reset service not available!")
            return
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


# ── Gymnasium Environment ─────────────────────────────────────────────────────
class BalanceEnv(gym.Env):
    """
    Observation:  [pitch, pitch_vel, left_thigh, left_knee, right_thigh, right_knee]
    Action:       [left_thigh_delta, left_knee_delta, right_thigh_delta, right_knee_delta, wheel_speed]
    Reward:       alive bonus - tilt penalty - wheel penalty - joint delta penalty - fall penalty
    Done:         |pitch| > FALL_PITCH_THRESH or steps > MAX_EPISODE_STEPS
    """

    metadata = {"render_modes": []}

    def __init__(self, config=None):
        super().__init__()

        self.observation_space = spaces.Box(
            low=np.array(OBS_LOW,  dtype=np.float32),
            high=np.array(OBS_HIGH, dtype=np.float32),
            dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=np.array(ACT_LOW,  dtype=np.float32),
            high=np.array(ACT_HIGH, dtype=np.float32),
            dtype=np.float32
        )

        # Init ROS2 once per environment instance
        if not rclpy.ok():
            rclpy.init()

        self.bridge   = RobotBridge()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)

        self._step_count = 0
        self._spin(duration=1.0)   # initial spin to get first sensor data

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _spin(self, duration: float):
        """Spin ROS2 executor for `duration` seconds to process callbacks."""
        end = time.time() + duration
        while time.time() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def _wait_for_sensors(self):
        """Block until both IMU and joint states are received."""
        while not (self.bridge.imu_ready and self.bridge.js_ready):
            self._spin(0.1)

    # ── reset ─────────────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0

        # 1. Stop wheels
        self.bridge.stop()

        # 2. Reset Gazebo physics + model pose
        self.bridge.reset_simulation()
        self._spin(0.5)   # let Gazebo settle

        # 3. Send stand pose
        self.bridge.send_joints(STAND_POSE, duration_sec=STAND_SETTLE_SEC)
        self._spin(STAND_SETTLE_SEC + 0.5)

        # 4. Wait for fresh sensor data
        self._wait_for_sensors()

        obs  = self.bridge.get_observation()
        info = {}
        return obs, info

    # ── step ──────────────────────────────────────────────────────────────────
    def step(self, action: np.ndarray):
        self._step_count += 1

        # Unpack action
        joint_deltas = action[:4]   # [lthigh, lknee, rthigh, rknee]
        wheel_speed  = float(action[4])

        # Build corrected joint targets = stand pose + deltas
        targets = {
            "left_thigh_joint":  STAND_POSE["left_thigh_joint"]  + float(joint_deltas[0]),
            "left_knee_joint":   STAND_POSE["left_knee_joint"]   + float(joint_deltas[1]),
            "right_thigh_joint": STAND_POSE["right_thigh_joint"] + float(joint_deltas[2]),
            "right_knee_joint":  STAND_POSE["right_knee_joint"]  + float(joint_deltas[3]),
        }

        # Apply action
        self.bridge.send_joints(targets, duration_sec=STEP_DURATION_SEC)
        self.bridge.send_cmd_vel(wheel_speed)

        # Wait one step duration for physics to evolve
        self._spin(STEP_DURATION_SEC)

        # Read new state
        obs   = self.bridge.get_observation()
        pitch = self.bridge.pitch

        # ── Reward ────────────────────────────────────────────────────────────
        fell = abs(pitch) > FALL_PITCH_THRESH

        reward = (
            REWARD["alive"]
            + REWARD["tilt"]         * abs(pitch)
            + REWARD["wheel"]        * abs(wheel_speed)
            + REWARD["joint_delta"]  * float(np.sum(np.abs(joint_deltas)))
            + (REWARD["fall"] if fell else 0.0)
        )

        # ── Termination ───────────────────────────────────────────────────────
        terminated = fell
        truncated  = self._step_count >= MAX_EPISODE_STEPS

        info = {
            "pitch":       pitch,
            "step":        self._step_count,
            "fell":        fell,
        }

        return obs, reward, terminated, truncated, info

    # ── cleanup ───────────────────────────────────────────────────────────────
    def close(self):
        self.bridge.stop()
        self.executor.shutdown()
        self.bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
