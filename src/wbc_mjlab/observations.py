"""Current left/right gait phase shared by observations and contact rewards."""
from __future__ import annotations
from typing import TYPE_CHECKING, cast
import torch
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand
if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv
LOCO_GAIT_PERIOD = 1.0
LOCO_GAIT_OFFSET = 0.5
LOCO_STANCE_RATIO = 0.55

def get_velocity_command(
  env: ManagerBasedRlEnv, command_name: str = "twist"
) -> UniformVelocityCommand:
  return cast(UniformVelocityCommand, env.command_manager.get_term(command_name))


def loco_leg_phase(
  env: ManagerBasedRlEnv,
  command_name: str = "twist",
  gait_period: float = LOCO_GAIT_PERIOD,
  gait_offset: float = LOCO_GAIT_OFFSET,
) -> torch.Tensor:
  walk_phase = (env.episode_length_buf.to(torch.float32) * env.step_dt) % gait_period
  walk_phase = walk_phase / gait_period
  twist_cmd = get_velocity_command(env, command_name)
  left = torch.where(twist_cmd.is_standing_env, torch.zeros_like(walk_phase), walk_phase)
  right = torch.where(
    twist_cmd.is_standing_env,
    torch.zeros_like(walk_phase),
    (walk_phase + gait_offset) % 1.0,
  )
  return torch.stack((left, right), dim=-1)


def loco_phase_features(
  env: ManagerBasedRlEnv,
  command_name: str = "twist",
  gait_period: float = LOCO_GAIT_PERIOD,
  gait_offset: float = LOCO_GAIT_OFFSET,
) -> torch.Tensor:
  phase = loco_leg_phase(
    env,
    command_name=command_name,
    gait_period=gait_period,
    gait_offset=gait_offset,
  )
  two_pi_phase = 2.0 * torch.pi * phase
  return torch.cat(
    (
      torch.sin(two_pi_phase[:, 0:1]),
      torch.cos(two_pi_phase[:, 0:1]),
      torch.sin(two_pi_phase[:, 1:2]),
      torch.cos(two_pi_phase[:, 1:2]),
    ),
    dim=-1,
  )
