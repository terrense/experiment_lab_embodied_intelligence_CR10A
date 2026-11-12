"""Agent policies for the inspection task (SIMULATION ONLY).

Every policy has the same interface, so the comparison is controlled:

    policy.reset(observation)          # at the start of an episode
    policy.act(observation) -> {"skill": str, "target_index": int | None}

The policies, in the order the decisive experiment compares them:

    A0  ScriptedFSM      a hand-written state machine with fixed recovery rules. **This is the
                         strong baseline**: if a learned policy does not beat it, the learning
                         claim is void. It is written to be competent, not to be bad.
    A1  PromptAgent      an LLM with no training, given the tools and the state as a prompt.
    A2  SFTAgent         an LLM fine-tuned on successful tool-call sequences.
    A4  GRPOAgent        an LLM optimised with GRPO/RLVR on the calibrated verifier reward.

A1, A2 and A4 need a language model. The model is reached through a `LanguageModel` object, so
that the SAME harness runs the scripted baseline without one. The LLM backends are deliberately
thin and are NOT part of this repository's default path: an API key is read from the environment
at run time and never committed.

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import json
import os
import re
from typing import Callable, Dict, List, Optional

#: the tools an agent may call, and what each one needs
TOOL_SCHEMA = [
    {"name": "plan_motion", "args": ["target_index"],
     "description": "plan a collision-free path to the target; returns plan_failed if none exists"},
    {"name": "execute_path", "args": [],
     "description": "follow the current path under the safety layer; returns blocked if the "
                    "safety layer stops the execution"},
    {"name": "inspect_state", "args": [],
     "description": "read the clearance, the distance to the target and the last tool outcome"},
    {"name": "replan", "args": ["target_index"],
     "description": "discard the current path and plan again for the target"},
    {"name": "retreat", "args": [],
     "description": "move away from the closest obstacle"},
    {"name": "finish", "args": [],
     "description": "declare the task complete"},
]


class AgentPolicy:
    """The interface every policy implements."""

    name = "policy"

    def reset(self, obs: Dict) -> None:
        pass

    def act(self, obs: Dict) -> Dict:
        raise NotImplementedError


class ScriptedFSM(AgentPolicy):
    """A0: the strong rule-based baseline.

    The rules, in the order they are tried:

    1. no path yet            -> plan_motion(current target)
    2. the plan failed       -> replan once; if that fails too, retreat and then plan again
    3. the execution blocked  -> retreat, then replan
    4. the target was moved   -> replan (the old path is stale)
    5. close to the target    -> inspect_state, to hold still and complete the inspection
    6. otherwise              -> execute_path

    The recovery budget is deliberately finite: after three failed plans on the same target the
    FSM gives up on that target and moves to the next one. A policy that never gives up would
    look better on "inspections done" and much worse on "tool calls", and the reward prices both.
    """

    name = "fsm"

    def __init__(self, max_plans_per_target: int = 3) -> None:
        self.max_plans_per_target = max_plans_per_target
        self.plans_on_target = 0
        self.needs_replan = True
        self.retreat_needed = False
        self.holding = False

    def reset(self, obs: Dict) -> None:
        self.plans_on_target = 0
        self.needs_replan = True
        self.retreat_needed = False
        self.holding = False

    def act(self, obs: Dict) -> Dict:
        status = obs.get("last_status", "")
        reason = obs.get("last_failure", "")
        target = obs.get("target_index", 0)

        if status == "plan_failed":
            self.plans_on_target += 1
            if self.plans_on_target >= self.max_plans_per_target:
                # give up on this target and move on
                self.needs_replan = True
                self.plans_on_target = 0
                return {"skill": "inspect_state", "target_index": target}
            self.needs_replan = True
            return {"skill": "replan", "target_index": target}
        if status == "blocked":
            self.retreat_needed = True
            self.needs_replan = True
            return {"skill": "retreat", "target_index": target}
        if status == "stale":
            self.needs_replan = True
            return {"skill": "replan", "target_index": target}

        if self.retreat_needed:
            self.retreat_needed = False
            return {"skill": "retreat", "target_index": target}
        if self.needs_replan:
            self.needs_replan = False
            self.plans_on_target += 1
            return {"skill": "plan_motion", "target_index": target}
        # close to the target: hold still so the inspection can complete
        if obs.get("distance_to_target_m", 1.0) < 0.02 and obs.get("hold_steps", 0) < 4:
            return {"skill": "inspect_state", "target_index": target}
        return {"skill": "execute_path", "target_index": target}


# --------------------------------------------------------------------------- the LLM path
PROMPT_TEMPLATE = """You are the controller of a 6-axis robot arm (Dobot CR10A) that must visit three
inspection targets in order, going around a wall that stands between the start and the targets.

TOOLS:
{tools}

STATE:
{state}

The last tool call returned: status={status}{reason}

Choose exactly one tool to call now. Answer with a single line:
TOOL: <name> [target_index]
"""


def render_prompt(obs: Dict) -> str:
    """The prompt an LLM policy is given. It is deliberately short and structured."""
    tools = "\n".join(f"  - {t['name']}({', '.join(t['args']) if t['args'] else ''}): {t['description']}"
                      for t in TOOL_SCHEMA)
    reason = f" ({obs.get('last_failure')})" if obs.get("last_failure") else ""
    return PROMPT_TEMPLATE.format(tools=tools, state=obs.get("text", ""),
                                   status=obs.get("last_status", "none"), reason=reason)


def parse_tool_call(text: str) -> Optional[Dict]:
    """Parse `TOOL: name [index]` out of a model answer."""
    m = re.search(r"TOOL:\s*([a-z_]+)", text or "", re.IGNORECASE)
    if not m:
        return None
    name = m.group(1).lower()
    if name not in {t["name"] for t in TOOL_SCHEMA}:
        return None
    idx = re.search(r"\[?\s*(\d+)\s*\]?", text[m.end():])
    return {"skill": name, "target_index": int(idx.group(1)) if idx else None}


class LanguageModel:
    """A thin wrapper over a chat completion API.

    The key is read from the environment at run time (`EX_LEARNING_LLM_API_KEY`) and is NEVER
    committed. The base URL and the model name can be overridden, so the same harness runs
    against a local model served on the compute server.
    """

    def __init__(self, model: Optional[str] = None, base_url: Optional[str] = None,
                 api_key: Optional[str] = None, temperature: float = 0.0,
                 max_tokens: int = 64) -> None:
        self.model = model or os.environ.get("EX_LEARNING_LLM_MODEL", "gpt-4o-mini")
        self.base_url = base_url or os.environ.get("EX_LEARNING_LLM_BASE_URL") or None
        self.api_key = api_key or os.environ.get("EX_LEARNING_LLM_API_KEY")
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.calls = 0
        self.total_tokens = 0

    @property
    def available(self) -> bool:
        return bool(self.api_key)

    def chat(self, prompt: str, system: str = "") -> str:
        if not self.api_key:
            raise RuntimeError("no language model configured: set EX_LEARNING_LLM_API_KEY")
        import urllib.request
        url = (self.base_url or "https://api.openai.com/v1") + "/chat/completions"
        body = {"model": self.model, "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "messages": ([{"role": "system", "content": system}] if system else [])
                           + [{"role": "user", "content": prompt}]}
        req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json",
                                              "Authorization": f"Bearer {self.api_key}"})
        with urllib.request.urlopen(req, timeout=60) as r:
            data = json.loads(r.read())
        self.calls += 1
        self.total_tokens += data.get("usage", {}).get("total_tokens", 0)
        return data["choices"][0]["message"]["content"]


class PromptAgent(AgentPolicy):
    """A1: an LLM with no training."""

    name = "prompt"

    def __init__(self, llm: LanguageModel, system: str = "") -> None:
        self.llm = llm
        self.system = system

    def reset(self, obs: Dict) -> None:
        pass

    def act(self, obs: Dict) -> Dict:
        text = self.llm.chat(render_prompt(obs), self.system)
        call = parse_tool_call(text)
        if call is None:
            # an unparseable answer is a wasted tool call, not a free pass: the harness counts it
            return {"skill": "inspect_state", "target_index": obs.get("target_index", 0),
                    "unparsed": True}
        if call["target_index"] is None:
            call["target_index"] = obs.get("target_index", 0)
        return call


class ScriptedPolicy(AgentPolicy):
    """A scripted policy that replays a list of tool calls - the data source for SFT (A2)."""

    name = "scripted"

    def __init__(self, script: List[Dict]) -> None:
        self.script = script
        self.i = 0

    def reset(self, obs: Dict) -> None:
        self.i = 0

    def act(self, obs: Dict) -> Dict:
        if self.i >= len(self.script):
            return {"skill": "finish", "target_index": obs.get("target_index", 0)}
        call = dict(self.script[self.i])
        self.i += 1
        call.setdefault("target_index", obs.get("target_index", 0))
        return call
