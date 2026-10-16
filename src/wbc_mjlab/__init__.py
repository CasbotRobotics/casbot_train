"""Register the standalone CASBOT02 locomotion teacher task."""
from mjlab.tasks.registry import register_mjlab_task
from mjlab.rl import MjlabOnPolicyRunner
from wbc_mjlab.casbot02_config import casbot02_loco_teacher_env_cfg
from wbc_mjlab.casbot02_rl_cfg import casbot02_loco_teacher_runner_cfg

register_mjlab_task(
  task_id="Casbot02-Loco-Teacher-Flat",
  env_cfg=casbot02_loco_teacher_env_cfg(play=False),
  play_env_cfg=casbot02_loco_teacher_env_cfg(play=True),
  rl_cfg=casbot02_loco_teacher_runner_cfg(),
  runner_cls=MjlabOnPolicyRunner,
)
