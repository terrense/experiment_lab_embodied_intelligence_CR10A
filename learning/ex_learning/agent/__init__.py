"""The agent layer: skills, task, verifier, harness and policies (SIMULATION ONLY).

**PARKED EXPLORATION - NOT PART OF THE PAPER.** On 2026-11-20 the owner set this direction
aside: the obstacle-avoidance line and the hardware get all the resources. The package is kept
because the measurements are real, the code is tested, and a portfolio should show what was
tried; the hypothesis it was built for was never tested, because no learned policy was trained.
See `docs/AGENT_EXPLORATION.md`.

It turns the platform's measured execution layer into the substrate of a high-level agent:

    skills.py     the robot's capabilities as callable, verifiable tools
    task.py       the multi-waypoint inspection task, with calibrated fault injection
    verifier.py   the calibrated reward: a physical event, a one-sided proxy and a censoring
                  correction, not a boolean
    policies.py   A0 (the strong rule-based baseline), A1 (prompt), A2 (SFT), A4 (GRPO)
    harness.py    the one loop every policy runs through

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from .harness import run_benchmark, run_episode  # noqa: F401
from .policies import (AgentPolicy, LanguageModel, PromptAgent,  # noqa: F401
                       ScriptedFSM, ScriptedPolicy, TOOL_SCHEMA, parse_tool_call, render_prompt)
from .skills import STATUSES, SkillLayer, SkillResult  # noqa: F401
from .task import FaultSpec, InspectionTask  # noqa: F401
from .verifier import EpisodeRecord, Verifier, summarise  # noqa: F401
