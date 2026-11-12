"""The agent benchmark: the DECISIVE experiment of the new main line (SIMULATION ONLY).

What this script does
---------------------
It runs one or more agent policies on the SAME episodes of the multi-waypoint inspection task
and scores them with the calibrated verifier. The policies are:

    fsm      A0: a hand-written state machine with fixed recovery rules. **The strong baseline.**
             If a learned policy does not beat it, the learning claim is void.
    prompt   A1: an LLM with no training (needs EX_LEARNING_LLM_API_KEY).
    scripted A2's data source: a scripted policy that replays a recorded tool-call sequence.

The episode records it writes are the unit an SFT dataset (A2) or a GRPO rollout set (A4) is
built from: the tool calls with their structured outcomes, the fault that fired, and the
verifier's verdict.

The "unseen fault combinations" evaluation
------------------------------------------
`--fault-profile seen` uses the fault probabilities the agent has been trained (or prompted)
with; `--fault-profile unseen` uses a DIFFERENT set - the same fault TYPES at different
probabilities and a different order - so that a policy cannot pass by memorising which fault
comes next. This is the comparison that decides the paper.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/agent_benchmark.py --policies fsm \
        --episodes 50 --fault-profile unseen

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from ex_learning.agent import (FaultSpec, InspectionTask, ScriptedFSM, SkillLayer,  # noqa: E402
                               Verifier, summarise)

#: the fault profiles. "seen" is what the agent is developed against; "unseen" uses the same
#: fault TYPES at different rates and a different order, so a policy that memorises the order
#: does not pass.
FAULT_PROFILES: Dict[str, FaultSpec] = {
    "seen": FaultSpec(plan_failure=0.10, target_moved=0.15, stale_feedback=0.10,
                      blocked_execution=0.05),
    "unseen": FaultSpec(plan_failure=0.22, target_moved=0.05, stale_feedback=0.18,
                        blocked_execution=0.12),
    "none": FaultSpec(plan_failure=0.0, target_moved=0.0, stale_feedback=0.0,
                      blocked_execution=0.0),
}


def make_task(args):
    """The environment, the task and the skill layer of one episode."""
    from ex_learning.envs import make_env
    env = make_env(args.task, conservative_geometry=True, substep_collision_check=True)
    env.cfg.max_steps = args.max_steps
    faults = FAULT_PROFILES[args.fault_profile]
    task = InspectionTask(env, n_targets=args.targets, faults=faults)
    skills = SkillLayer(env, margin=args.margin, plan_checks=args.plan_checks,
                        plan_candidates=args.plan_candidates, plan_speed=args.plan_speed,
                        use_filter=not args.no_filter)
    return env, task, skills


def policy_factory(args, name: str):
    if name == "fsm":
        return lambda: ScriptedFSM(max_plans_per_target=args.max_plans_per_target)
    if name == "prompt":
        from ex_learning.agent import LanguageModel, PromptAgent
        llm = LanguageModel(model=args.llm_model, temperature=args.llm_temperature)
        if not llm.available:
            raise SystemExit("the prompt policy needs EX_LEARNING_LLM_API_KEY in the environment")
        return lambda: PromptAgent(llm)
    if name == "scripted":
        from ex_learning.agent import ScriptedPolicy
        script = json.loads(Path(args.script).read_text()) if args.script else []
        return lambda: ScriptedPolicy(script)
    raise SystemExit(f"unknown policy {name!r}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--policies", nargs="*", default=["fsm"])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed0", type=int, default=20000)
    ap.add_argument("--targets", type=int, default=3)
    ap.add_argument("--fault-profile", default="seen", choices=sorted(FAULT_PROFILES))
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--plan-checks", type=int, default=20_000)
    ap.add_argument("--plan-candidates", type=int, default=4)
    ap.add_argument("--plan-speed", type=float, default=0.6)
    ap.add_argument("--max-steps", type=int, default=300)
    ap.add_argument("--max-plans-per-target", type=int, default=3)
    ap.add_argument("--no-filter", action="store_true",
                    help="run the skills WITHOUT the safety layer (the ablation)")
    ap.add_argument("--llm-model", default=None)
    ap.add_argument("--llm-temperature", type=float, default=0.0)
    ap.add_argument("--script", default=None, help="a JSON file with the scripted tool calls")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "agent_benchmark.json")
    ap.add_argument("--episodes-out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "agent_episodes.json")
    a = ap.parse_args(argv)

    from ex_learning.agent import run_episode

    verifier = Verifier()
    out: Dict = {"task": a.task, "episodes": a.episodes, "seed0": a.seed0,
                 "fault_profile": a.fault_profile, "targets": a.targets,
                 "margin": a.margin, "filter": not a.no_filter,
                 "policies": {}}
    episodes_out: Dict = {}

    for name in a.policies:
        t0 = time.time()
        verdicts = []
        records = []
        for ep in range(a.episodes):
            env, task, skills = make_task(a)
            policy = policy_factory(a, name)()
            rec = run_episode(env, task, skills, policy, a.seed0 + ep, episode=ep)
            verdicts.append(verifier.evaluate(rec))
            records.append({"episode": ep, "seed": rec.seed,
                            "skill_calls": rec.skill_calls,
                            "faults": getattr(rec, "faults", []),
                            "reward": verdicts[-1]["reward"],
                            "reward_terms": verdicts[-1]["reward_terms"],
                            "inspections_done": rec.inspections_done,
                            "envelope_contacts": rec.envelope_contacts,
                            "min_mesh_clearance_m": verdicts[-1]["min_mesh_clearance_m"]})
        summary = summarise(verdicts)
        summary["wall_s"] = round(time.time() - t0, 1)
        out["policies"][name] = {"summary": summary, "verdicts": verdicts}
        episodes_out[name] = records
        print(f"\n== {name}: completion {summary['completion_rate']*100:.0f} %   "
              f"inspections {summary['mean_inspections']:.2f}/3   "
              f"mesh contact {summary['mesh_collision_rate']*100:.0f} %   "
              f"envelope contact {summary['envelope_contact_rate']*100:.0f} %   "
              f"tool calls {summary['mean_tool_calls']:.1f}   "
              f"invalid {summary['mean_invalid_calls']:.1f}   "
              f"reward {summary['mean_reward']:+.2f}   "
              f"min mesh clearance {1000*summary['min_min_mesh_clearance_m']:.0f} mm   "
              f"({summary['wall_s']:.0f} s)")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    a.episodes_out.parent.mkdir(parents=True, exist_ok=True)
    a.episodes_out.write_text(json.dumps(episodes_out, indent=1))
    print(f"\nwrote {a.out}\nwrote {a.episodes_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
