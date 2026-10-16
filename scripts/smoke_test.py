"""Short headless environment/PPO test; output is diagnostic, not a trained policy."""
import argparse
from dataclasses import asdict
from pathlib import Path
import tempfile
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from wbc_mjlab.casbot02_config import casbot02_loco_teacher_env_cfg
from wbc_mjlab.casbot02_rl_cfg import casbot02_loco_teacher_runner_cfg


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--device", default="cuda:0")
  parser.add_argument("--ppo", action="store_true", help="Train one iteration and save a diagnostic checkpoint")
  parser.add_argument("--output-dir", type=Path)
  args = parser.parse_args()
  cfg = casbot02_loco_teacher_env_cfg()
  cfg.scene.num_envs = 2
  cfg.scene.terrain.num_envs = 2
  env = ManagerBasedRlEnv(cfg=cfg, device=args.device)
  try:
    wrapper = RslRlVecEnvWrapper(env)
    obs = wrapper.get_observations()
    assert obs["actor"].shape == (2, 196), obs["actor"].shape
    assert obs["critic"].shape == (2, 208), obs["critic"].shape
    for _ in range(5):
      obs, reward, done, extras = wrapper.step(torch.zeros(2, 12, device=args.device))
      assert torch.isfinite(reward).all()
      assert all(torch.isfinite(v).all() for v in obs.values())
    print("Environment smoke test passed: obs=196/208, action=12, finite rewards.")
    if args.ppo:
      output = args.output_dir or Path(tempfile.mkdtemp(prefix="casbot02-smoke-"))
      output.mkdir(parents=True, exist_ok=True)
      runner = MjlabOnPolicyRunner(wrapper, asdict(casbot02_loco_teacher_runner_cfg()), log_dir=str(output), device=args.device)
      runner.learn(num_learning_iterations=1, init_at_random_ep_len=False)
      print(f"PPO smoke checkpoint directory: {output.resolve()}")
  finally:
    env.close()


if __name__ == "__main__":
  main()
