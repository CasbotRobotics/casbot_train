"""Casbot02 loco teacher: 196 actor observations and 12 leg actions."""

from __future__ import annotations

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as env_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import (
  ObservationGroupCfg,
  ObservationTermCfg,
)
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import (
  ContactMatch,
  ContactSensorCfg,
  ObjRef,
  RingPatternCfg,
  TerrainHeightSensorCfg,
)
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from wbc_mjlab import casbot02_constants as C
from wbc_mjlab import rewards as wbc_rewards
from wbc_mjlab import observations as wbc_obs
from wbc_mjlab.casbot02_actions import Casbot02LegWithArmSwingActionCfg


def _leg_asset() -> SceneEntityCfg:
  """SceneEntityCfg selecting only the 12 policy-controlled leg joints."""
  return SceneEntityCfg(
    "robot", joint_names=C.CASBOT02_LEG_ONLY_JOINT_NAMES, preserve_order=True
  )


def _leg_actuator_asset() -> SceneEntityCfg:
  """SceneEntityCfg selecting only the 12 policy-controlled leg actuators."""
  return SceneEntityCfg(
    "robot", actuator_names=C.CASBOT02_LEG_ONLY_JOINT_NAMES, preserve_order=True
  )


def _leg_joint_and_actuator_asset() -> SceneEntityCfg:
  """12 policy-controlled legs for joint-velocity × actuator-force terms."""
  return SceneEntityCfg(
    "robot",
    joint_names=C.CASBOT02_LEG_ONLY_JOINT_NAMES,
    actuator_names=C.CASBOT02_LEG_ONLY_JOINT_NAMES,
    preserve_order=True,
  )


_ANKLE_PITCH_JOINT_WEIGHTS: tuple[float, ...] = tuple(
  1.0 if name in ("leg_l5_joint", "leg_r5_joint") else 0.0
  for name in C.CASBOT02_LEG_ONLY_JOINT_NAMES
)


def _torso_asset() -> SceneEntityCfg:
  return SceneEntityCfg("robot", body_names=("torso",))


def _feet_body_asset() -> SceneEntityCfg:
  return SceneEntityCfg(
    "robot", body_names=("leg_l6_link", "leg_r6_link"), preserve_order=True
  )


def _feet_site_asset() -> SceneEntityCfg:
  return SceneEntityCfg(
    "robot", site_names=("left_foot", "right_foot"), preserve_order=True
  )


def _force_site_asset() -> SceneEntityCfg:
  """Select the ankle-origin sites used by AMP_mjlab's slip reward."""
  return SceneEntityCfg(
    "robot", site_names=("left_force", "right_force"), preserve_order=True
  )


def _knee_body_asset() -> SceneEntityCfg:
  """Select knee and hip-yaw bodies in the order required by the reward."""
  return SceneEntityCfg(
    "robot",
    body_names=("leg_l4_link", "leg_l3_link", "leg_r4_link", "leg_r3_link"),
    preserve_order=True,
  )


def _roll_yaw_actuator_asset() -> SceneEntityCfg:
  """Select the lateral/yaw leg actuators that produced internal torque."""
  return SceneEntityCfg(
    "robot",
    actuator_names=(
      "leg_l2_joint",  # hip roll
      "leg_l3_joint",  # hip yaw
      "leg_l6_joint",  # ankle roll
      "leg_r2_joint",
      "leg_r3_joint",
      "leg_r6_joint",
    ),
    preserve_order=True,
  )


def _roll_yaw_joint_asset() -> SceneEntityCfg:
  """Select the six measured non-sagittal leg joint positions."""
  return SceneEntityCfg(
    "robot",
    joint_names=(
      "leg_l2_joint",  # hip roll
      "leg_l3_joint",  # hip yaw
      "leg_l6_joint",  # ankle roll
      "leg_r2_joint",
      "leg_r3_joint",
      "leg_r6_joint",
    ),
    preserve_order=True,
  )


def _loco_actor_terms(enable_noise: bool) -> dict[str, ObservationTermCfg]:
  """45-dim actor obs, matching the AMP teacher's layout + noise."""
  return {
    "base_ang_vel": ObservationTermCfg(
      func=env_mdp.builtin_sensor,
      params={"sensor_name": "robot/angular-velocity"},
      noise=Unoise(n_min=-0.2, n_max=0.2) if enable_noise else None,
    ),
    "projected_gravity": ObservationTermCfg(
      func=env_mdp.projected_gravity,
      noise=Unoise(n_min=-0.05, n_max=0.05) if enable_noise else None,
    ),
    "command": ObservationTermCfg(
      func=env_mdp.generated_commands,
      params={"command_name": "twist"},
    ),
    "joint_pos": ObservationTermCfg(
      func=env_mdp.joint_pos_rel,
      params={"asset_cfg": _leg_asset()},
      noise=Unoise(n_min=-0.01, n_max=0.01) if enable_noise else None,
    ),
    "joint_vel": ObservationTermCfg(
      func=env_mdp.joint_vel_rel,
      params={"asset_cfg": _leg_asset()},
      noise=Unoise(n_min=-0.5, n_max=0.5) if enable_noise else None,
    ),
    "actions": ObservationTermCfg(func=env_mdp.last_action),
  }


def _feet_contact_sensor() -> ContactSensorCfg:
  return ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree", pattern=r"^(leg_l6_link|leg_r6_link)$", entity="robot"
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )


def _self_collision_sensor() -> ContactSensorCfg:
  return ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="torso", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="torso", entity="robot"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )


def _apply_casbot02_robot(cfg: ManagerBasedRlEnvCfg) -> None:
  """Swap in the Casbot02 robot, add its sensors, wire action + viewer."""
  cfg.scene.entities = {"robot": C.get_casbot02_23dof_robot_cfg()}

  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    _feet_contact_sensor(),
    _self_collision_sensor(),
  )

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  cfg.actions["joint_pos"] = Casbot02LegWithArmSwingActionCfg(
    entity_name="robot",
    actuator_names=C.CASBOT02_LEG_ONLY_JOINT_NAMES,
    scale=C.CASBOT02_LEG_ONLY_ACTION_SCALE,
    use_default_offset=True,
  )

  cfg.viewer.body_name = "torso"


def _apply_flat_terrain(cfg: ManagerBasedRlEnvCfg) -> None:
  """Flat terrain: plane, no terrain generator / height sensors."""
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  for sensor in cfg.scene.sensors:
    if sensor.name == "foot_height_scan":
      assert isinstance(sensor, TerrainHeightSensorCfg)
      sensor.frame = tuple(
        ObjRef(type="site", name=s, entity="robot")
        for s in ("left_foot", "right_foot")
      )
      sensor.pattern = RingPatternCfg.single_ring(radius=0.03, num_samples=6)

  cfg.curriculum.pop("terrain_levels", None)


def _apply_loco_rewards(cfg: ManagerBasedRlEnvCfg) -> None:
  """Casbot02 velocity-tracking reward stack with G1 foot shaping."""

  cfg.rewards["action_rate_l2"].weight = -0.1
  cfg.rewards.pop("ankle_action_rate_l2", None)

  cfg.rewards.pop("swing_height_curve", None)
  cfg.rewards["foot_clearance"] = RewardTermCfg(
    func=velocity_mdp.feet_clearance,
    weight=-6.0,
    params={
      "height_sensor_name": "foot_height_scan",
      "target_height": 0.07,
      "command_name": "twist",
      "command_threshold": 0.05,
      "asset_cfg": _feet_site_asset(),
    },
  )

  cfg.rewards["foot_swing_height"] = RewardTermCfg(
    func=velocity_mdp.feet_swing_height,
    weight=-0.75,
    params={
      "sensor_name": "feet_ground_contact",
      "height_sensor_name": "foot_height_scan",
      "target_height": 0.10,
      "command_name": "twist",
      "command_threshold": 0.05,
    },
  )
  cfg.rewards["foot_slip"].weight = -0.3
  cfg.rewards["foot_slip"].params["command_threshold"] = 0.05
  cfg.rewards["foot_slip"].params["asset_cfg"] = _feet_site_asset()
  cfg.rewards["soft_landing"].weight = -2e-4

  cfg.rewards["air_time"].weight = 0.0
  cfg.rewards["turning_air_time"] = RewardTermCfg(
    func=wbc_rewards.command_conditioned_feet_air_time,
    weight=0.0,
    params={
      "sensor_name": "feet_ground_contact",
      "command_name": "twist",
      "threshold_min": 0.05,
      "translation_threshold_max": 0.60,
      "turning_threshold_max": 0.42,
      "command_threshold": 0.2,
      "turning_linear_threshold": 0.2,
      "turning_angular_threshold": 0.2,
      "turning_only": True,
    },
  )
  cfg.rewards["feet_air_time"] = RewardTermCfg(
    func=wbc_rewards.feet_air_time,
    weight=0.0,
    params={
      "sensor_name": "feet_ground_contact",
      "uniform_cmd_name": "twist",
      "feet_air_time_target": None,
    },
  )

  cfg.rewards["gait_phase_contact"] = RewardTermCfg(
    func=wbc_rewards.gait_phase_contact,
    weight=1.0,
    params={
      "sensor_name": "feet_ground_contact",
      "command_name": "twist",
      "gait_period": 1.0,
      "gait_offset": 0.5,
      "stance_ratio": 0.55,
    },
  )

  cfg.rewards["track_linear_velocity"].params["asset_cfg"] = _torso_asset()
  cfg.rewards["track_linear_velocity"].params["std"] = 0.5
  cfg.rewards["track_angular_velocity"].params["asset_cfg"] = _torso_asset()
  cfg.rewards["track_angular_velocity"].params["std"] = 0.7071
  cfg.rewards["upright"].params["asset_cfg"] = _torso_asset()
  cfg.rewards["pose"].weight = 1.0
  cfg.rewards["pose"].params["asset_cfg"] = _leg_asset()
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    r"leg_[lr]1_joint": 0.30,   # hip pitch  0.30 * 1.73/2.43
    r"leg_[lr]2_joint": 0.05,   # hip roll   0.15 * 0.175/0.524
    r"leg_[lr]3_joint": 0.15,  # hip yaw    0.15 * 1.57/2.76
    r"leg_[lr]4_joint": 0.35,   # knee       0.35 * 0.36/0.387
    r"leg_[lr]5_joint": 0.25,   # ankle pitch ~same travel as G1
    r"leg_[lr]6_joint": 0.10,   # ankle roll 0.10 * 0.506/0.262
  }
  cfg.rewards["pose"].params["std_running"] = {
    r"leg_[lr]1_joint": 0.36,   # hip pitch  0.50 * 1.73/2.43
    r"leg_[lr]2_joint": 0.07,   # hip roll   0.20 * 0.175/0.524
    r"leg_[lr]3_joint": 0.11,   # hip yaw    0.20 * 1.57/2.76
    r"leg_[lr]4_joint": 0.56,   # knee       0.60 * 0.36/0.387
    r"leg_[lr]5_joint": 0.35,   # ankle pitch ~same travel as G1
    r"leg_[lr]6_joint": 0.29,   # ankle roll 0.15 * 0.506/0.262
  }
  cfg.rewards["stand_pose"] = RewardTermCfg(
    func=wbc_rewards.stand_pose,
    weight=-3.0,
    params={
      "command_name": "twist",
      "asset_cfg": _leg_asset(),
    },
  )
  cfg.rewards["feet_distance_lateral"] = RewardTermCfg(
    func=wbc_rewards.feet_distance_lateral,
    weight=1,
    params={
      "asset_cfg": _feet_site_asset(),
      "min_distance": 0.255,
      "max_distance": 0.42,
    },
  )
  cfg.rewards["knee_distance_lateral"] = RewardTermCfg(
    func=wbc_rewards.knee_distance_lateral,
    weight=2,
    params={
      "asset_cfg": _knee_body_asset(),
      "min_distance": 0.255,
      "max_distance": 0.42,
    },
  )
  cfg.rewards["flat_foot"] = RewardTermCfg(
    func=wbc_rewards.flat_foot,
    weight=-0.5,
    params={
      "sensor_name": "feet_ground_contact",
      "asset_cfg": _feet_body_asset(),
    },
  )
  cfg.rewards["body_ang_vel"].weight = -0.1
  cfg.rewards["body_ang_vel"].params["asset_cfg"] = _torso_asset()
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["linear_momentum_change"] = RewardTermCfg(
    func=wbc_rewards.LinearMomentumChangePenalty,
    weight=1e-6,
    params={},
  )
  cfg.rewards["angular_momentum_change"] = RewardTermCfg(
    func=wbc_rewards.AngularMomentumChangePenalty,
    weight=1e-5,
    params={},
  )
  cfg.rewards["torque_limits"] = RewardTermCfg(
    func=wbc_rewards.applied_torque_limits_by_ratio,
    weight=-0.01,
    params={
      "asset_cfg": _leg_actuator_asset(),
      "limit_ratio": 0.8,
    },
  )
  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=velocity_mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": "self_collision", "force_threshold": 10.0},
  )


def _apply_casbot02_dr(cfg: ManagerBasedRlEnvCfg) -> None:
  """Casbot02 域随机化, 对齐 amp_mjlab(补足 make_velocity_env_cfg 里空转的项)."""
  cfg.events["foot_friction"].params["ranges"] = (0.3, 1.2)
  cfg.events["foot_friction"].params["asset_cfg"] = SceneEntityCfg(
    "robot", geom_names=C.CASBOT02_FOOT_GEOM_NAMES, preserve_order=True
  )

  cfg.events["base_mass"] = EventTermCfg(
    mode="startup",
    func=dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names=("torso",)),
      "ranges": (-4.0, 4.0),
      "operation": "add",
      "distribution": "uniform",
    },
  )

  cfg.events["actuator_gains"] = EventTermCfg(
    mode="startup",
    func=dr.pd_gains,
    params={
      "asset_cfg": SceneEntityCfg("robot", actuator_ids=[0, 1]),
      "kp_range": (0.85, 1.15),
      "kd_range": (0.85, 1.15),
      "operation": "scale",
      "distribution": "uniform",
    },
  )

  cfg.events["joint_friction"] = EventTermCfg(
    mode="startup",
    func=dr.joint_friction,
    params={
      "asset_cfg": _leg_asset(),
      "ranges": (0.5, 1.5),
      "operation": "scale",
      "distribution": "uniform",
    },
  )

  cfg.events["base_com"].params["asset_cfg"] = SceneEntityCfg(
    "robot", body_names=("torso",)
  )

  cfg.events["joint_default_pos"] = EventTermCfg(
    mode="startup",
    func=dr.joint_default_pos,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      "ranges": (-0.02, 0.02),
      "distribution": "uniform",
      "operation": "add",
    },
  )

  cfg.events["non_base_mass"] = EventTermCfg(
    mode="startup",
    func=dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names=(r"^(?!torso$).+$",)),
      "ranges": (0.80, 1.20),
      "operation": "scale",
      "distribution": "uniform",
      "shared_random": False,
    },
  )



def _apply_twist_ranges(cfg: ManagerBasedRlEnvCfg) -> None:
  """G1 sampling with a fixed backward limit and no lateral commands."""
  base_twist_cmd = cfg.commands["twist"]
  assert isinstance(base_twist_cmd, UniformVelocityCommandCfg)
  vx, vy, wz = (-0.5, 1.0), (0.0, 0.0), (-1.0, 1.0)
  twist_cmd = UniformVelocityCommandCfg(
    resampling_time_range=base_twist_cmd.resampling_time_range,
    debug_vis=base_twist_cmd.debug_vis,
    entity_name=base_twist_cmd.entity_name,
    heading_command=base_twist_cmd.heading_command,
    heading_control_stiffness=base_twist_cmd.heading_control_stiffness,
    rel_standing_envs=0.1,
    rel_heading_envs=0.3,
    rel_world_envs=0.0,
    rel_forward_envs=0.2,
    init_velocity_prob=0.0,
    ranges=UniformVelocityCommandCfg.Ranges(
      lin_vel_x=vx,
      lin_vel_y=vy,
      ang_vel_z=wz,
      heading=base_twist_cmd.ranges.heading,
    ),
    viz=base_twist_cmd.viz,
  )
  cfg.commands["twist"] = twist_cmd

  # Expand forward speed and yaw rate at 5000 iterations (24 steps each).
  # Keep the backward limit at -0.5 m/s and vy at zero throughout training.
  cfg.curriculum["command_vel"].params["velocity_stages"] = [
    {
      "step": 0,
      "lin_vel_x": (vx[0], vx[1] * 0.5),
      "lin_vel_y": vy,
      "ang_vel_z": (wz[0] * 0.5, wz[1] * 0.5),
    },
    {
      "step": 5000 * 24,
      "lin_vel_x": vx,
      "lin_vel_y": vy,
      "ang_vel_z": wz,
    },
  ]


def _apply_leg_obs(
  cfg: ManagerBasedRlEnvCfg, *, play: bool, include_phase: bool = True
) -> None:
  """Four-frame proprioception and optional four-frame gait-phase history."""
  enable_noise = not play
  actor_terms = _loco_actor_terms(enable_noise=enable_noise)
  for term in actor_terms.values():
    term.history_length = 4
  if include_phase:
    actor_terms["phase"] = ObservationTermCfg(
      func=wbc_obs.loco_phase_features,
      params={"command_name": "twist", "gait_period": 1.0, "gait_offset": 0.5},
      history_length=4,
    )
  cfg.observations["actor"] = ObservationGroupCfg(
    terms=actor_terms,
    concatenate_terms=True,
    enable_corruption=enable_noise,
    history_length=None,
  )
  # Critic = actor terms + privileged base_lin_vel (deployment-invisible).
  critic_terms = _loco_actor_terms(enable_noise=False)
  for term in critic_terms.values():
    term.history_length = 4
  if include_phase:
    critic_terms["phase"] = ObservationTermCfg(
      func=wbc_obs.loco_phase_features,
      params={"command_name": "twist", "gait_period": 1.0, "gait_offset": 0.5},
      history_length=4,
    )
  critic_terms["base_lin_vel"] = ObservationTermCfg(
    func=env_mdp.builtin_sensor,
    params={"sensor_name": "robot/linear-velocity"},
    history_length=4,
  )
  cfg.observations["critic"] = ObservationGroupCfg(
    terms=critic_terms,
    concatenate_terms=True,
    enable_corruption=False,
    history_length=None,
  )


def casbot02_loco_teacher_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """12-leg loco teacher with gait phase (actor 196, critic 208, action 12)."""
  cfg = make_velocity_env_cfg()

  _apply_casbot02_robot(cfg)
  _apply_flat_terrain(cfg)
  _apply_loco_rewards(cfg)
  _apply_twist_ranges(cfg)
  _apply_leg_obs(cfg, play=play)

  if play:
    for name in (
      "foot_friction", "encoder_bias", "base_com", "base_mass",
      "actuator_gains", "joint_friction", "joint_default_pos",
      "non_base_mass", "push_robot",
    ):
      cfg.events.pop(name, None)
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.curriculum = {}
  else:
    cfg.seed = 42
    cfg.scene.num_envs = 4096
    assert cfg.scene.terrain is not None
    cfg.scene.terrain.num_envs = 4096
    _apply_casbot02_dr(cfg)

  return cfg

