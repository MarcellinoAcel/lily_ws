# ─────────────────────────────────────────────────────────────────────────────
# config.py  —  All tunable parameters in one place
# ─────────────────────────────────────────────────────────────────────────────

# ── ROS2 Topics ───────────────────────────────────────────────────────────────
TOPICS = {
    "imu":          "/imu/data",
    "joint_states": "/joint_states",
    "cmd_vel":      "/cmd_vel",
    "action_server":"/leg_controller/follow_joint_trajectory",
}

# ── Gazebo Services ───────────────────────────────────────────────────────────
SERVICES = {
    "reset":        "/reset_simulation",         # std_srvs/srv/Empty
    "set_state":    "/gazebo/set_model_state",   # gazebo_msgs/srv/SetModelState
    "model_name":   "bipedal_robot",             # TODO: match your model name in Gazebo
}

# ── Stand Pose (absolute joint positions) ─────────────────────────────────────
# TODO: confirm left_thigh_joint value
STAND_POSE = {
    "left_thigh_joint":  -2.512,
    "left_knee_joint":   -1.587,
    "right_thigh_joint":  2.512,
    "right_knee_joint":  -1.587,
}

JOINT_NAMES = [
    "left_thigh_joint",
    "left_knee_joint",
    "right_thigh_joint",
    "right_knee_joint",
]

# ── Observation Space ─────────────────────────────────────────────────────────
# [ pitch, pitch_velocity, left_thigh, left_knee, right_thigh, right_knee ]
OBS_DIM = 6

OBS_LOW  = [-1.57, -5.0, -3.14, -3.14, -3.14, -3.14]
OBS_HIGH = [ 1.57,  5.0,  3.14,  3.14,  3.14,  3.14]

# ── Action Space ──────────────────────────────────────────────────────────────
# [ left_thigh_delta, left_knee_delta, right_thigh_delta, right_knee_delta, wheel_speed ]
ACT_DIM = 5

ACT_LOW  = [-0.3, -0.3, -0.3, -0.3, -0.5]
ACT_HIGH = [ 0.3,  0.3,  0.3,  0.3,  0.5]

# ── Episode ───────────────────────────────────────────────────────────────────
MAX_EPISODE_STEPS  = 200        # max steps before timeout
STEP_DURATION_SEC  = 0.05       # seconds per step (50ms)
STAND_SETTLE_SEC   = 2.0        # wait after reset before episode starts
FALL_PITCH_THRESH  = 0.5        # radians — episode ends if |pitch| > this

# ── Reward Weights ────────────────────────────────────────────────────────────
REWARD = {
    "alive":         1.0,    # given every step robot stays upright
    "tilt":         -5.0,    # multiplied by |pitch|
    "wheel":        -0.1,    # multiplied by |wheel_speed|
    "joint_delta":  -0.05,   # multiplied by sum of |joint deltas|
    "fall":       -100.0,    # one-time penalty when episode ends by falling
}

# ── RLlib / PPO Hyperparameters ───────────────────────────────────────────────
RLLIB = {
    "framework":        "torch",
    "num_workers":      2,          # parallel Gazebo instances (1 per worker)
    "num_envs_per_worker": 1,
    "rollout_fragment_length": 200,
    "train_batch_size": 4000,
    "sgd_minibatch_size": 128,
    "num_sgd_iter":     10,
    "lr":               3e-4,
    "gamma":            0.99,
    "lambda":           0.95,
    "clip_param":       0.2,
    "vf_clip_param":    10.0,
    "entropy_coeff":    0.01,       # encourages exploration
    "model": {
        "fcnet_hiddens":  [256, 256],
        "fcnet_activation": "tanh",
    },
}

CHECKPOINT_DIR  = "./checkpoints"
CHECKPOINT_FREQ = 10    # save every N iterations
MAX_ITERATIONS  = 500
