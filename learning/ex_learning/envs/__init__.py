"""Task registry: one place that maps a task name to an environment."""

TASKS = ("reach", "wall", "wall_obs", "window", "wall_rand", "wall_rand_blind", "wall_ood", "wall_ood_blind")


def make_env(task: str, seed: int = 0, **overrides):
    """Build the environment of a task.

    `overrides` are applied to the task's config (e.g. `conservative_geometry=True` for the
    corrected collision evaluator). The default is always the LEGACY configuration, so the
    published numbers stay reproducible.
    """
    if task == "reach":
        from .cr10a_reach import ReachConfig, ReachEnv
        cfg = ReachConfig(**{k: v for k, v in overrides.items() if hasattr(ReachConfig(), k)})
        return ReachEnv(cfg, seed=seed)
    from .cr10a_reach_wall import ReachWallEnv, WallConfig
    if task == "wall":
        cfg = WallConfig()
    elif task == "wall_obs":  # the original fixed wall, observed like the random ones (for wall_rand policies)
        cfg = WallConfig(wall_in_obs=True)
    elif task == "window":
        # narrow passage: start on the robot side, goal behind a wall with a 0.50 x 0.40 m window;
        # mesh point clouds instead of enclosing capsules (capsules cannot thread the window)
        # shaping_margin 2 cm = the pools' minimum clearance: v1 used 5 cm and 68 % of the goals sit 2-5 cm from
        # the window frame, so standing AT the goal was penalised every step (reward contradicted the task)
        cfg = WallConfig(scene="window", query=None, collision_model="points", shaping_margin=0.02)
    elif task.startswith(("wall_rand", "wall_ood")):
        # generalisation: a new wall every episode. *_blind = the wall is NOT in the observation
        # (for policies trained on the fixed wall, obs 22); otherwise its centre + half sizes are (obs 28)
        from .cr10a_reach_wall import OOD_WALLS, TRAIN_WALLS
        ranges = OOD_WALLS if task.startswith("wall_ood") else TRAIN_WALLS
        cfg = WallConfig(wall_ranges=ranges, wall_in_obs=not task.endswith("_blind"))
    else:
        raise ValueError(f"task must be one of {TASKS}, got {task!r}")
    for k, v in overrides.items():
        if not hasattr(cfg, k):
            raise ValueError(f"{task} has no config field {k!r}")
        setattr(cfg, k, v)
    return ReachWallEnv(cfg, seed=seed)
