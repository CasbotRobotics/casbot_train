"""Reward helpers used by the Casbot02 loco teacher."""
from __future__ import annotations
from typing import TYPE_CHECKING
import torch
from mjlab.entity import Entity
from mjlab.envs import mdp as env_mdp
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor
from mjlab.utils.lab_api.math import quat_apply_inverse
from wbc_mjlab.observations import LOCO_STANCE_RATIO, loco_leg_phase
if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv
_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")

def _get_robot(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> Entity:
  return env.scene[asset_cfg.name]


def _body_masses(asset: Entity) -> torch.Tensor:
  """Per-entity-body masses. Shape [num_envs, n_bodies]."""
  return asset.data.model.body_mass[:, asset.data.indexing.body_ids]


def _linear_momentum(asset: Entity) -> torch.Tensor:
  """Total linear momentum p = Σ m_i * v_i. Shape [num_envs, 3]."""
  masses = _body_masses(asset)
  return (masses.unsqueeze(-1) * asset.data.body_com_lin_vel_w).sum(dim=1)


def _angular_momentum(asset: Entity) -> torch.Tensor:
  """Orbital angular momentum L = Σ (r_i − r_CoM) × (m_i v_i).

  Spin term (I_i ω_i) is omitted for GPU performance (~10–20 % error vs full
  CAM). Shape [num_envs, 3].
  """
  masses = _body_masses(asset)
  total_mass = masses.sum(dim=1, keepdim=True).clamp_min(1e-8)
  body_pos = asset.data.body_com_pos_w
  body_vel = asset.data.body_com_lin_vel_w
  com_pos = (masses.unsqueeze(-1) * body_pos).sum(dim=1) / total_mass
  rel_pos = body_pos - com_pos.unsqueeze(1)
  return torch.linalg.cross(
    rel_pos, masses.unsqueeze(-1) * body_vel, dim=-1
  ).sum(dim=1)


class LinearMomentumChangePenalty:
  """Penalize rate of change of whole-body linear momentum (F = dp/dt).

  Returns a *negative* squared penalty; pair with a positive weight.
  Spin angular momentum is excluded (orbital term only).
  """

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv) -> None:
    self._asset_cfg: SceneEntityCfg = cfg.params.get("asset_cfg", _DEFAULT_ASSET_CFG)
    self._env = env
    asset = _get_robot(env, self._asset_cfg)
    self._prev_lin_mom: torch.Tensor = _linear_momentum(asset).clone()

  def __call__(self, env: ManagerBasedRlEnv) -> torch.Tensor:
    asset = _get_robot(env, self._asset_cfg)
    cur = _linear_momentum(asset)
    net_force = (cur - self._prev_lin_mom) / env.step_dt
    self._prev_lin_mom = cur.clone()
    return -torch.sum(net_force**2, dim=-1)

  def reset(self, env_ids: torch.Tensor | slice | None) -> None:
    asset = _get_robot(self._env, self._asset_cfg)
    full = _linear_momentum(asset)
    if env_ids is None:
      self._prev_lin_mom = full.clone()
    else:
      self._prev_lin_mom[env_ids] = full[env_ids]


class AngularMomentumChangePenalty:
  """Penalize rate of change of whole-body angular momentum (τ = dL/dt).

  Uses orbital angular momentum only (no body spin term).
  Returns a *negative* squared penalty; pair with a positive weight.
  """

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv) -> None:
    self._asset_cfg: SceneEntityCfg = cfg.params.get("asset_cfg", _DEFAULT_ASSET_CFG)
    self._env = env
    asset = _get_robot(env, self._asset_cfg)
    self._prev_ang_mom: torch.Tensor = _angular_momentum(asset).clone()

  def __call__(self, env: ManagerBasedRlEnv) -> torch.Tensor:
    asset = _get_robot(env, self._asset_cfg)
    cur = _angular_momentum(asset)
    net_torque = (cur - self._prev_ang_mom) / env.step_dt
    self._prev_ang_mom = cur.clone()
    return -torch.sum(net_torque**2, dim=-1)

  def reset(self, env_ids: torch.Tensor | slice | None) -> None:
    asset = _get_robot(self._env, self._asset_cfg)
    full = _angular_momentum(asset)
    if env_ids is None:
      self._prev_ang_mom = full.clone()
    else:
      self._prev_ang_mom[env_ids] = full[env_ids]


def _expand_gravity_for_bodies(
  gravity_vec_w: torch.Tensor, *, num_envs: int, num_bodies: int
) -> torch.Tensor:
  """Broadcast gravity to `[num_envs, num_bodies, 3]` for body-wise projections."""
  if gravity_vec_w.ndim == 1:
    gravity_vec_w = gravity_vec_w.unsqueeze(0)

  if gravity_vec_w.shape[0] == 1 and num_envs != 1:
    gravity_vec_w = gravity_vec_w.expand(num_envs, -1)
  elif gravity_vec_w.shape[0] != num_envs:
    raise ValueError(
      f"Expected gravity_vec_w to have {num_envs} rows, got {gravity_vec_w.shape[0]}"
    )

  return gravity_vec_w.unsqueeze(1).expand(-1, num_bodies, -1)


def command_turning_blend(
  command: torch.Tensor,
  *,
  turning_linear_threshold: float,
  turning_angular_threshold: float,
) -> torch.Tensor:
  """Return a smooth 0=translation, 1=in-place-turn command blend.

  A pure turn with ``|wz| >= turning_angular_threshold`` receives 1.  The
  blend fades to zero as linear speed reaches ``turning_linear_threshold``;
  normal curved walking therefore keeps the translational gait target.
  """
  if turning_linear_threshold <= 0.0 or turning_angular_threshold <= 0.0:
    raise ValueError("Turning blend thresholds must be positive")
  linear_norm = torch.norm(command[:, :2], dim=1)
  angular_norm = torch.abs(command[:, 2])
  low_translation = torch.clamp(
    1.0 - linear_norm / turning_linear_threshold, min=0.0, max=1.0
  )
  active_turn = torch.clamp(
    angular_norm / turning_angular_threshold, min=0.0, max=1.0
  )
  return low_translation * active_turn


def command_conditioned_feet_air_time(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  threshold_min: float = 0.05,
  translation_threshold_max: float = 0.60,
  turning_threshold_max: float = 0.42,
  command_name: str = "twist",
  command_threshold: float = 0.2,
  turning_linear_threshold: float = 0.2,
  turning_angular_threshold: float = 0.2,
  turning_only: bool = False,
) -> torch.Tensor:
  """Dense air-time reward with a shorter window for in-place turns.

  With ``turning_only=True``, scale the reward by the command's turn blend so
  straight translation receives no contribution.
  """
  sensor: ContactSensor = env.scene[sensor_name]
  current_air_time = sensor.data.current_air_time
  assert current_air_time is not None
  command = env.command_manager.get_command(command_name)
  assert command is not None
  turn_blend = command_turning_blend(
    command,
    turning_linear_threshold=turning_linear_threshold,
    turning_angular_threshold=turning_angular_threshold,
  )
  threshold_max = torch.lerp(
    torch.full_like(turn_blend, translation_threshold_max),
    torch.full_like(turn_blend, turning_threshold_max),
    turn_blend,
  ).unsqueeze(1)
  in_range = (current_air_time > threshold_min) & (
    current_air_time < threshold_max
  )
  reward = torch.sum(in_range.float(), dim=1)
  linear_norm = torch.norm(command[:, :2], dim=1)
  angular_norm = torch.abs(command[:, 2])
  active = (linear_norm + angular_norm > command_threshold).float()

  in_air = current_air_time > 0.0
  num_in_air = torch.clamp(torch.sum(in_air.float()), min=1.0)
  env.extras["log"]["Metrics/air_time_mean"] = (
    torch.sum(current_air_time * in_air.float()) / num_in_air
  )
  env.extras["log"]["Metrics/air_time_max_target_mean"] = threshold_max.mean()
  if turning_only:
    reward = reward * turn_blend
  return reward * active


def stand_pose(
  env: ManagerBasedRlEnv,
  command_name: str = "twist",
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  asset = _get_robot(env, asset_cfg)
  default_joint_pos = asset.data.default_joint_pos
  joint_ids = asset_cfg.joint_ids
  error = torch.sum(
    torch.square(asset.data.joint_pos[:, joint_ids] - default_joint_pos[:, joint_ids]),
    dim=-1,
  )
  twist_cmd = env.command_manager.get_term(command_name)
  return error * twist_cmd.is_standing_env.float()


def flat_foot(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  contact_force_threshold: float = 1.0,
) -> torch.Tensor:
  asset = _get_robot(env, asset_cfg)
  sensor: ContactSensor = env.scene[sensor_name]
  force = sensor.data.force
  assert force is not None
  contact = (torch.norm(force[..., :3], dim=-1) > contact_force_threshold).float()
  foot_quat = asset.data.body_link_quat_w[:, asset_cfg.body_ids, :]
  gravity = _expand_gravity_for_bodies(
    asset.data.gravity_vec_w,
    num_envs=env.num_envs,
    num_bodies=foot_quat.shape[1],
  )
  projected = quat_apply_inverse(
    foot_quat.reshape(-1, 4), gravity.reshape(-1, 3)
  ).reshape(env.num_envs, len(asset_cfg.body_ids), 3)
  tilt_error = torch.sum(torch.square(projected[..., :2]), dim=-1)
  return torch.sum(tilt_error * contact, dim=-1)


def gait_phase_contact(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  command_name: str = "twist",
  gait_period: float = 1.0,
  gait_offset: float = 0.5,
  stance_ratio: float = LOCO_STANCE_RATIO,
  contact_force_threshold: float = 1.0,
) -> torch.Tensor:
  sensor: ContactSensor = env.scene[sensor_name]
  force = sensor.data.force
  assert force is not None
  contact = torch.norm(force[..., :3], dim=-1) > contact_force_threshold
  expected_stance = loco_leg_phase(
    env,
    command_name=command_name,
    gait_period=gait_period,
    gait_offset=gait_offset,
  ) < stance_ratio
  return (~(contact ^ expected_stance)).float().sum(dim=-1)


def feet_distance_lateral(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg,
  min_distance: float,
  max_distance: float,
) -> torch.Tensor:
  """Reward zero inside a lateral foot-spacing band and negative outside it.

  The two feet may be selected by either body names (the original G1 usage)
  or site names.  Site positions are preferable for Casbot02 because its
  ``left_foot``/``right_foot`` sites lie at the collision-sole centers.
  """
  asset = _get_robot(env, asset_cfg)
  root_quat = asset.data.root_link_quat_w
  if asset_cfg.site_names is not None:
    foot_pos_w = asset.data.site_pos_w[:, asset_cfg.site_ids, :]
  else:
    foot_pos_w = asset.data.body_link_pos_w[:, asset_cfg.body_ids, :]
  if foot_pos_w.shape[1] != 2:
    raise ValueError(
      "feet_distance_lateral requires exactly two foot bodies or sites, "
      f"got {foot_pos_w.shape[1]}"
    )

  left_right_delta_w = foot_pos_w[:, 0] - foot_pos_w[:, 1]
  left_right_delta_b = quat_apply_inverse(root_quat, left_right_delta_w)
  lateral = torch.abs(left_right_delta_b[:, 1])
  too_close = torch.clamp(lateral - min_distance, max=0.0)
  too_far = torch.clamp(-lateral + max_distance, max=0.0)
  return too_close + too_far


def knee_distance_lateral(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg,
  min_distance: float,
  max_distance: float,
) -> torch.Tensor:
  asset = _get_robot(env, asset_cfg)
  root_pos = asset.data.root_link_pos_w
  root_quat = asset.data.root_link_quat_w
  body_pos = asset.data.body_link_pos_w[:, asset_cfg.body_ids, :]
  delta = body_pos - root_pos.unsqueeze(1)
  body_pos_b = quat_apply_inverse(
    root_quat.unsqueeze(1).expand(-1, len(asset_cfg.body_ids), -1).reshape(-1, 4),
    delta.reshape(-1, 3),
  ).reshape(env.num_envs, len(asset_cfg.body_ids), 3)
  lateral = torch.abs(body_pos_b[:, 0, 1] - body_pos_b[:, 2, 1]) + torch.abs(
    body_pos_b[:, 1, 1] - body_pos_b[:, 3, 1]
  )
  too_close = torch.clamp(lateral - 2.0 * min_distance, max=0.0)
  too_far = torch.clamp(-lateral + 2.0 * max_distance, max=0.0)
  return too_close + too_far


def applied_torque_limits_by_ratio(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  limit_ratio: float = 0.8,
) -> torch.Tensor:
  """Penalize Σ (|τ| - ratio·τ_max)²_+ over selected actuators.

  Ported from InstinctLab parkour (``mdp.applied_torque_limits_by_ratio``).
  mjlab has no ``applied_torque`` / ``joint_effort_limits`` fields, so the
  actuator-force output stands in for the applied torque and the simulation
  ``actuator_forcerange`` upper bound stands in for the per-actuator effort
  limit. Only the excess above ``limit_ratio`` of the limit is penalized.
  """
  asset = _get_robot(env, asset_cfg)
  applied_torque = torch.abs(asset.data.actuator_force[:, asset_cfg.actuator_ids])
  force_range = env.sim.model.actuator_forcerange
  if force_range.ndim == 2:
    max_force = force_range[asset_cfg.actuator_ids, 1]
  else:
    max_force = force_range[:, asset_cfg.actuator_ids, 1]
  max_force = torch.clamp(max_force, min=1.0e-6)
  out_of_limits = torch.clamp(applied_torque - max_force * limit_ratio, min=0.0)
  return torch.sum(torch.square(out_of_limits), dim=-1)


class feet_air_time:
  """HANDOFF landing-time reward gated by reference motion speed."""

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    del cfg
    self.step_dt = env.step_dt

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    sensor_name: str,
    command_name: str = "motion",
    feet_air_time_target: float | None = 0.5,
    uniform_cmd_name: str | None = None,
  ) -> torch.Tensor:
    sensor: ContactSensor = env.scene[sensor_name]
    last_air_time = sensor.data.last_air_time
    assert last_air_time is not None
    first_contact = sensor.compute_first_contact(dt=self.step_dt).float()
    if feet_air_time_target is None:
      # No target: reward any lift proportional to its air time (longer stride
      # = more reward), no upper cap. Encourages step length to scale with speed.
      air_time = last_air_time
    else:
      # Penalize shortfall below the target at landing (drives air time -> target).
      air_time = torch.clamp(last_air_time - feet_air_time_target, max=0.0)
    reward = torch.sum(air_time * first_contact, dim=1)
    if uniform_cmd_name is not None:
      uni_xy = env_mdp.generated_commands(env, command_name=uniform_cmd_name)[:, :2]
      active = torch.norm(uni_xy, dim=1) > 0.05
    else:
      raise ValueError("Set uniform_cmd_name=twist for the loco-only task")
    return reward * active.float()

  def reset(self, env_ids: torch.Tensor | slice | None) -> None:
    del env_ids
