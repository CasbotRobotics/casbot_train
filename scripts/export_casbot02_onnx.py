"""Export a CASBOT02 loco checkpoint with normalization included."""
from __future__ import annotations
import argparse
from dataclasses import asdict
from pathlib import Path
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from wbc_mjlab.casbot02_config import casbot02_loco_teacher_env_cfg
from wbc_mjlab.casbot02_rl_cfg import casbot02_loco_teacher_runner_cfg


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("checkpoint", type=Path)
  parser.add_argument("--output", type=Path, help="Default: checkpoint directory / <run name>.onnx")
  args = parser.parse_args()
  if not args.checkpoint.is_file():
    parser.error(f"checkpoint not found: {args.checkpoint}")
  output = args.output or args.checkpoint.parent / f"{args.checkpoint.parent.name}.onnx"
  output.parent.mkdir(parents=True, exist_ok=True)
  env = ManagerBasedRlEnv(cfg=casbot02_loco_teacher_env_cfg(play=True), device="cpu")
  try:
    runner = MjlabOnPolicyRunner(RslRlVecEnvWrapper(env), asdict(casbot02_loco_teacher_runner_cfg()), device="cpu")
    runner.load(str(args.checkpoint))
    runner.export_policy_to_onnx(str(output.parent), filename=output.name)
    print(f"ONNX exported: {output.resolve()}")
  finally:
    env.close()


if __name__ == "__main__":
  main()
