"""PPO and symmetry configuration for the Casbot02 loco teacher."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def _mlp_actor() -> RslRlModelCfg:
  """Plain MLP actor (512/256/128 -> 12), matching the AMP teacher's arch."""
  return RslRlModelCfg(
    hidden_dims=(512, 256, 128),
    activation="elu",
    obs_normalization=True,
    distribution_cfg={
      "class_name": "GaussianDistribution",
      "init_std": 1.0,
      "std_type": "scalar",
    },
  )


def _mlp_critic() -> RslRlModelCfg:
  return RslRlModelCfg(
    hidden_dims=(512, 256, 128),
    activation="elu",
    obs_normalization=True,
  )


@dataclass
class Casbot02PpoAlgorithmCfg(RslRlPpoAlgorithmCfg):
  """PPO config exposing RSL-RL's optional symmetry extension."""

  symmetry_cfg: dict[str, Any] | None = None


def _ppo_algorithm(
  class_name: str = "rsl_rl.algorithms:PPO",
  *,
  use_loco_symmetry: bool = False,
) -> Casbot02PpoAlgorithmCfg:
  symmetry_cfg = None
  if use_loco_symmetry:
    symmetry_cfg = {
      "data_augmentation_func": (
        "wbc_mjlab.casbot02_symmetry:compute_symmetric_states"
      ),
      "use_data_augmentation": True,
      "use_mirror_loss": True,
      "mirror_loss_coeff": 0.1,
    }
  return Casbot02PpoAlgorithmCfg(
    class_name=class_name,
    num_learning_epochs=5,
    num_mini_batches=4,
    learning_rate=1.0e-3,
    schedule="adaptive",
    gamma=0.99,
    lam=0.95,
    entropy_coef=0.005,
    desired_kl=0.01,
    max_grad_norm=1.0,
    value_loss_coef=1.0,
    use_clipped_value_loss=True,
    clip_param=0.2,
    symmetry_cfg=symmetry_cfg,
  )


def casbot02_loco_teacher_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Plain-PPO runner for the 12-leg Casbot02 velocity-tracking loco teacher."""
  return RslRlOnPolicyRunnerCfg(
    seed=42,
    num_steps_per_env=24,
    max_iterations=20_001,
    obs_groups={"actor": ("actor",), "critic": ("critic",)},
    save_interval=1000,
    experiment_name="casbot02_loco_teacher",
    run_name="casbot02_loco_teacher",
    logger="tensorboard",
    wandb_project="casbot02",
    actor=_mlp_actor(),
    critic=_mlp_critic(),
    algorithm=_ppo_algorithm(use_loco_symmetry=True),
  )

