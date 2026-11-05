"""Measure the command channel of the SIMULATED plant (SIMULATION ONLY).

This is the simulation-side counterpart of the hardware step the research plan calls "characterise
the command interface": how does a requested joint velocity become executed motion, and how far
does the arm coast after the command is cut?

Three numbers are measured on the MuJoCo plant (not from the model):

  1. the STEADY-STATE tracking lag: the distance between the command and the executed motion
     while the command is held;
  2. the COAST: the distance travelled after the command is cut to zero - the quantity a
     kinematic safety filter ignores entirely;
  3. the multi-step PREDICTION ERROR of the fitted execution model, which is what a filter built
     on that model inherits.

The output is `data/reports/learning/execution_channel.json`.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/measure_execution.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))

from ex_learning.envs import make_env  # noqa: E402
from ex_learning.execution_model import (ServoExecutionModel, _log_rollout,  # noqa: E402
                                         _prediction_error, coast_distance,
                                         identify_execution_model)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_rand")
    ap.add_argument("--horizon", type=int, default=5)
    ap.add_argument("--rollouts", type=int, default=3)
    ap.add_argument("--steps", type=int, default=90)
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "execution_channel.json")
    a = ap.parse_args(argv)

    env = make_env(a.task)
    model = ServoExecutionModel(dt=env.cfg.control_dt)
    out = {"task": a.task, "horizon_steps": a.horizon,
           "note": "SIMULATED plant. The real CR10A channel (queued MovJ + proprietary servo) "
                   "is a different object and none of these numbers transfer to it.",
           "rows": []}

    print("== the coast, measured on the plant ==")
    for v in (0.25, 0.5, 0.75, 1.0):
        vc = np.full(6, v)
        c = coast_distance(env, model, v=vc)
        out["rows"].append({"command_rad_s": v, **c})
        print(f"  command {v:4.2f} rad/s -> coast {c['coast_rad']*1000:6.2f} mrad "
              f"(ratio {c['coast_ratio']:.4f} rad per rad/s), model predicts "
              f"{c['pred_coast_rad']*1000:6.2f} mrad")

    print("\n== fitting the execution model ==")
    fitted, report = identify_execution_model(env, policy=None, n_rollouts=a.rollouts,
                                               n_steps=a.steps, horizon=a.horizon)
    out["identification"] = report
    print(f"  omega {fitted.omega:.2f} rad/s (gains say {2*np.pi*8:.2f}), zeta {fitted.zeta:.2f}, "
          f"v_gain {fitted.v_gain:.2f}")
    print(f"  multi-step prediction error over {a.horizon} steps: "
          f"RMS {report['prediction_error']['rms_rad']*1000:.2f} mrad")

    # the sensitivity of the error to a WRONG execution model: this is what decides whether the
    # filter's advantage survives a mis-calibrated channel
    print("\n== sensitivity to a wrong execution model ==")
    sens = []
    for scale in (0.6, 0.8, 1.0, 1.2, 1.5):
        m = ServoExecutionModel(omega=fitted.omega * scale, zeta=fitted.zeta,
                                v_gain=fitted.v_gain, dt=env.cfg.control_dt)
        errs = []
        for s in range(1000, 1000 + a.rollouts):
            rng = np.random.default_rng(s)
            log = _log_rollout(env, lambda obs, rng=rng: rng.uniform(-1.0, 1.0, 6), a.steps, s)
            errs.append(_prediction_error(m, log, np.asarray(env.lo, float),
                                          np.asarray(env.hi, float), a.horizon)["rms_rad"])
        e = float(np.mean(errs))
        sens.append({"omega_scale": scale, "omega": m.omega, "rms_rad": e})
        print(f"  omega x{scale:4.2f} -> RMS {e*1000:7.2f} mrad")
    out["sensitivity"] = sens

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
