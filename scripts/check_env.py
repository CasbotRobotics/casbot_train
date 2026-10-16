"""Check dependencies, GPU availability, task configuration and robot assets."""
import argparse
import importlib.metadata as metadata
import sys
import mujoco
import torch
from wbc_mjlab.casbot02_constants import get_spec
from wbc_mjlab.casbot02_config import casbot02_loco_teacher_env_cfg


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--require-cuda", action="store_true")
  args = parser.parse_args()
  print("Python:", sys.version.split()[0])
  for name in ("mjlab", "torch", "mujoco", "rsl-rl-lib", "onnx", "onnxruntime", "mujoco-python-viewer"):
    try:
      print(f"{name}: {metadata.version(name)}")
    except metadata.PackageNotFoundError:
      print(f"{name}: not installed (install the sim2sim extra for ONNX playback)")
  print("CUDA available:", torch.cuda.is_available())
  if torch.cuda.is_available():
    for i in range(torch.cuda.device_count()):
      print(f"GPU {i}: {torch.cuda.get_device_name(i)}")
  model = get_spec().compile()
  print(f"Robot assets OK: {model.njnt} joints, {model.nmesh} meshes")
  cfg = casbot02_loco_teacher_env_cfg()
  print("Task: Casbot02-Loco-Teacher-Flat; actor=196, critic=208, action=12")
  print("Configured parallel environments:", cfg.scene.num_envs)
  if args.require_cuda and not torch.cuda.is_available():
    raise SystemExit("CUDA unavailable. Check nvidia-smi and the NVIDIA driver before training.")


if __name__ == "__main__":
  main()
