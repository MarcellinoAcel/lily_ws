# ─────────────────────────────────────────────────────────────────────────────
# eval.py  —  Load a trained checkpoint and run it in Gazebo
# Run: python3 eval.py --checkpoint ./checkpoints/best
# ─────────────────────────────────────────────────────────────────────────────

import argparse
import numpy as np
import ray
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env

from lily_learning.lily_learning.balance_env import BalanceEnv
from config import CHECKPOINT_DIR


def env_creator(config):
    return BalanceEnv(config)

register_env("BipedalBalance-v0", env_creator)


def run_eval(checkpoint_path: str, num_episodes: int = 5, render: bool = True):
    ray.init(ignore_reinit_error=True)

    # Rebuild config (must match training config exactly)
    algo = (
        PPOConfig()
        .environment("BipedalBalance-v0")
        .framework("torch")
        .rollouts(num_rollout_workers=0)   # 0 = run env in main process
        .build()
    )

    algo.restore(checkpoint_path)
    print(f"\n✓ Loaded checkpoint: {checkpoint_path}")
    print(f"  Running {num_episodes} episodes...\n")

    env = BalanceEnv()

    for ep in range(num_episodes):
        obs, _ = env.reset()
        done        = False
        total_reward = 0.0
        steps        = 0
        max_pitch    = 0.0

        while not done:
            # Use deterministic=True for eval (no exploration noise)
            action = algo.compute_single_action(obs, deterministic=True)

            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            total_reward += reward
            steps        += 1
            max_pitch     = max(max_pitch, abs(info["pitch"]))

        fell = info.get("fell", False)
        print(f"Episode {ep+1:3d}  |  "
              f"reward: {total_reward:8.2f}  |  "
              f"steps: {steps:4d}  |  "
              f"max_pitch: {max_pitch:.3f} rad  |  "
              f"{'FELL' if fell else 'survived'}")

    env.close()
    algo.stop()
    ray.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=f"{CHECKPOINT_DIR}/best",
        help="Path to RLlib checkpoint directory"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="Number of evaluation episodes to run"
    )
    args = parser.parse_args()
    run_eval(args.checkpoint, num_episodes=args.episodes)
