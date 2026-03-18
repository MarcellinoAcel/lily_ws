# ─────────────────────────────────────────────────────────────────────────────
# train.py  —  RLlib PPO training script
# Run: python3 train.py
# ─────────────────────────────────────────────────────────────────────────────
import os
import ray
# from ray import tune
from ray.rllib.algorithms.ppo import PPOConfig
# from ray.tune.logger import pretty_print
from lily_learning.config import RLLIB, CHECKPOINT_DIR, CHECKPOINT_FREQ, MAX_ITERATIONS
from lily_learning.balance_env import BalanceEnv
# ── Register environment with RLlib ───────────────────────────────────────────
from ray.tune.registry import register_env
def env_creator(config):
    return BalanceEnv(config)
register_env("BipedalBalance-v0", env_creator)
# ── Build PPO config ──────────────────────────────────────────────────────────
def build_config() -> PPOConfig:
    cfg = (
        PPOConfig()
        .environment("BipedalBalance-v0")
        .framework(RLLIB["framework"])
        # Rollout workers — each worker runs its own Gazebo env
        # TODO: set num_env_runners=1 first, increase after confirming stability
        .env_runners(
            num_env_runners             = RLLIB["num_workers"],           # was: num_rollout_workers
            num_envs_per_env_runner     = RLLIB["num_envs_per_worker"],   # was: num_envs_per_worker
            rollout_fragment_length     = RLLIB["rollout_fragment_length"],
        )
        # PPO training settings
        .training(
            train_batch_size   = RLLIB["train_batch_size"],
            mini_batch_size_per_learner = RLLIB["mini_batch_size_per_learner"],
            num_sgd_iter       = RLLIB["num_sgd_iter"], 
            lr                 = RLLIB["lr"],
            gamma              = RLLIB["gamma"],
            lambda_            = RLLIB["lambda"],
            clip_param         = RLLIB["clip_param"],
            vf_clip_param      = RLLIB["vf_clip_param"],
            entropy_coeff      = RLLIB["entropy_coeff"],
            model              = RLLIB["model"],
        )
        # Evaluation
        .evaluation(
            evaluation_interval         = 10,   # evaluate every 10 iterations
            evaluation_num_workers      = 1,
            evaluation_duration         = 5,    # run 5 episodes per eval
        )
    )
    return cfg  
# ── Training loop ─────────────────────────────────────────────────────────────
def train():
    ray.init(ignore_reinit_error=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    algo = build_config().build()
    print("\n── Starting training ──────────────────────────────")
    print(f"   Max iterations : {MAX_ITERATIONS}")
    print(f"   Checkpoint dir : {CHECKPOINT_DIR}")
    print(f"   Workers        : {RLLIB['num_workers']}")
    print("───────────────────────────────────────────────────\n")
    best_reward = float("-inf")
    for i in range(MAX_ITERATIONS):
        result = algo.train()
        # New RLlib API nests metrics under "env_runners"
        env_runner_metrics = result.get("env_runners", result)
        mean_reward = env_runner_metrics.get("episode_reward_mean", float("nan"))
        ep_len      = env_runner_metrics.get("episode_len_mean", float("nan"))
        print(f"[Iter {i+1:4d}]  "
              f"reward: {mean_reward:8.2f}  |  "
              f"ep_len: {ep_len:6.1f}  |  "
              f"timesteps: {result['timesteps_total']:,}")
        # Save checkpoint periodically
        if (i + 1) % CHECKPOINT_FREQ == 0:
            path = algo.save(CHECKPOINT_DIR)
            print(f"  ✓ Checkpoint saved → {path}")
        # Save best model separately
        if mean_reward > best_reward:
            best_reward = mean_reward
            best_path = algo.save(os.path.join(CHECKPOINT_DIR, "best"))
            print(f"  ★ New best reward {best_reward:.2f} → {best_path}")
    print("\nTraining complete.")
    algo.stop()
    ray.shutdown()
if __name__ == "__main__":
    train()