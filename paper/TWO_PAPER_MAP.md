# Two-Paper Research Map | CR10A Robot Learning and Safety

**Status:** editorial research plan (2026-11-23), NOT a record of completed experiments.
**Decision owner:** principal investigator. **Reference source:** the unmodified historical paper/manuscript.md and existing simulation records.
**Research premise:** Paper I assumes known, registered obstacle geometry. Paper II relaxes that assumption; perception and uncertainty become the scientific subject.

## 1. Distinct and non-overlapping claims

| | Paper I — known geometry | Paper II — perception uncertainty |
|---|---|---|
| Scientific question | Can a predicted-motion action filter correctly characterize and improve the safety–completion trade-off of learned manipulator motion proposals under position-command execution? | How can a learned robot policy and predictive safety shield act responsibly when obstacle geometry is partial, delayed, or uncertain? |
| Geometry | Complete known obstacles, with prescribed geometry approximation error | Incomplete/occluded sensor-derived geometry, timestamp, confidence and unknown-space mask |
| Perception | OUT of scope | IN scope; simulator-derived depth/point clouds, with realistic errors |
| Learned policy | Existing PPO is a controlled nominal proposal; no new RL algorithm required to submit the baseline manuscript | Shield-conditioned policy learning is an explicit research contribution candidate |
| Safety | Existing bounded all-pair QP and short-horizon look-ahead filter; add stop-tail candidate verification if possible | Extend Paper I action/execution model to calibrated uncertainty, temporal validity and risk-aware fallback |
| Existing empirical evidence | Strong, but restricted to simulated, conservative envelope/termination semantics | No completed results yet |
| Primary success criterion | Honest, reproducible analysis plus measurable improvement relative to a strong safe QP under identical conditions | Joint improvement in coverage-calibrated risk, safe completion, intervention cost and OOD robustness relative to both Paper I and perception baselines |
| Claims NOT allowed | Guaranteed physical safety, real-world learned-policy deployment, superiority over safe QP completion, proof the coast term caused gains | Provable safety from uncalibrated prediction, reconstruction of occluded truth, general sim-to-real from simulation alone |

**Paper I proposed working title:** *Horizon-Constrained Safety Filtering for Learned Manipulator Motion: Execution Mismatch and Evaluation Fidelity*.

**Paper II proposed working title:** *Uncertainty-Calibrated Perception-to-Action Safety for Learned Manipulator Motion under Partial Observability*.

Both titles are provisional. In particular, the title of Paper I must not claim a validated execution-model advantage until the no-coast ablation changes under an independently tested setup.

## 2. What the historical manuscript actually established

The original source is paper/manuscript.md (approximately 97k characters; preserve it without deletion).

1. The archived single-constraint projection and detector required correction; all-pair bounded QP and corrected substep evaluation are stronger baselines.
2. A horizon-constrained filter was implemented and measured. The working implementation is a SINGLE candidate command with linearized look-ahead conditions; it is NOT a sequence MPC, stopping-tail optimizer or nonlinear candidate recheck.
3. In the frozen OOD test, three-seed aggregate: fixed-margin 4-cm bounded QP has 45.3% completion/0% corrected envelope contacts; 1-cm look-ahead 54.0%/0%; zero-margin QP 54.0%/19.3%. Those figures have different trade-off meanings.
4. Nine-policy-seed analysis does NOT establish a completion improvement against the *safe* 4-cm QP despite a mean positive difference. Do not convert the descriptive interval into a significance claim contradicting the archived verdict.
5. Removing the explicit coast term in the horizon method made no difference on the archived ablation. The effect is attributable to the horizon/constraint structure, NOT a validated execution-dynamics model contribution.
6. The filter considers a 0.1-s horizon while measured stopping motion lasts 0.2–0.5 s and can lose up to 87 mm clearance in simulation. Stop-tail verification is NOT implemented in its decision rule.
7. Collision data must separate **safety-envelope contact**, **physical mesh contact**, and **episode termination censoring**. Zero mesh contact on a trajectory stopped at envelope violation is not a meaningful uncensored zero-collision claim.
8. A matched no-privileged-goal classical planner reaches about 82% completion in an OOD comparison; nominal learning is not assumed superior.
9. No closed-loop robot hardware evidence exists for the learned policy / proposed filter. Controller consistency is not external metrology and simulator model identification is not real-hardware identification.

The rewritten Paper I draft lives in paper/paper1/manuscript.md. Numbers are retained only as retrospective archived measurements. New experiments must have new manifests and explicit provenance.

## 3. Migration from historical sections

| Historical manuscript | Paper I | Paper II | Action |
|---|---|---|---|
| Abstract / Introduction | Yes | Background only | Rewrite narrow question and measured contributions; remove unsupported execution-awareness causation |
| §2 Related work | Yes | New literature is needed | Expand with T-RO 2026 robust MPC, ICRA 2026 CBF-RL, T-RO 2026 SARA; prioritize relevant comparisons |
| §3 Platform | Yes | Reuse platform description with citation | Keep model geometry, servo chain, known geometry assumption |
| §4 Audit findings | Yes | Use only as prerequisite | Keep crucial bugs and collision-metric correction, move lengthy audit traces into supplemental material |
| §5.0–5.3 Proposed method | Yes | Reuse as baseline | Distinguish implemented bounded QP and current look-ahead |
| §5.4 Stopping tail / nonlinear verification | Planned extension only | Strong foundation of future robust filter | Do not present as implemented; require code and ablation before moving into final Paper I method |
| §5.5 Geometric argument | Restricted conditional statement | Extend conditional uncertainty arguments | Explicit assumptions; avoid unconditional certification |
| §5.6 Planner / optional residual RL | Planner as strong comparison | Learned shield-aware training reserved for Paper II | Remove unrun residual RL from first paper's main contribution |
| §6 Evaluation | Yes | Reuse reporting protocol, not results | Freeze methods, budgets, paired seeds, uncensored mesh checker |
| §7.1–7.12 Results | Yes, curated | No | Keep reproducible tables; move secondary audits to appendix; label all limitations |
| §8–9 Discussion/Limitations | Yes, rewritten | Motivation only | Separate established findings from future conjecture |
| Historical Agent / GRPO experiments | No | No | Leave code archived; no data or claims included |
| Sensor, point cloud, camera | No | Yes | Paper II owns perception errors and observation reliability |
| Diffusion expert trajectories / PPO-DAP | No mandatory use | Optional research branch only | Add only after a fair BC / retrieval baseline and a unique hypothesis |

## 4. Paper I contributions: what may be claimed vs what still needs proof

**C1 — Evaluation fidelity (measured):** a corrected, auditable protocol for conservative capsule contact, independent mesh checks and uncensored failure analysis. Valuable as reproducibility/methodology; do not mislabel standard collision geometry as a novel geometric theorem.

**C2 — Short-horizon filtering (measured):** bounded multi-pair look-ahead constraints are empirically useful relative to a weak zero-margin QP; the fair fixed-margin QP remains competitive. This alone is NOT yet a strong differentiating algorithmic advance over predictive controllers.

**C3 — Stop-tail and model-validity-aware candidate acceptance (proposed, NOT implemented):** include nonlinear rollout of candidate commands and complete backup stopping, track held-out prediction errors, reject uncertain/infeasible candidates and log fallback use. Only promote to an algorithm contribution after passing the frozen strong-baseline and ablation gates.

A lean publishable paper may emphasize careful empirical evaluation and a clearly scoped horizon filter. A stronger paper requires C3 to work; if the data fail, remove the novelty claim rather than manufacturing improvement.

## 5. Paper II journal hypotheses (each needs independent evidence)

**J1 — Spatiotemporal geometric belief:** transform partial observations into per-link clearance *intervals/lower bounds* and explicit unknown-space/age states. Novelty must be in calibration and consistency with decision-time uncertainty, not in merely mounting a camera.

**J2 — Validity-aware, risk-budgeted predictive shield:** propagate geometric uncertainty AND stopping/command uncertainty through the motion horizon, with certified-when-assumptions-hold and abstain/fallback behavior when confidence expires. Distinguish measured empirical coverage from deterministic guarantees.

**J3 — Shield-conditioned policy learning:** train a policy to act under partial and delayed geometry while minimizing correction, failure and unnecessary stop costs; compare against fixed margins, naive domain randomization, and post-hoc filtering. Critic/action semantics must be internally consistent.

One single coherent architecture should test J1, J2, J3; they must be independently ablated, not simply counted as three headings. A journal-level submission also needs broad geometries, multiple distribution shifts, strong planners and transparent resource/latency analysis. Second-robot or supervised real sensor validation is a valuable stretch, not an already completed claim.

## 6. Shared platform, strict boundary

Reusable package: robot model and kinematics; collision geometry; nominal controllers; bounded QP and predictor; common episode manifest and independent evaluator; logged command/position traces.

Do not reuse Paper I experimental outcomes as if they were Paper II results. Paper II should include a fully re-executed Paper-I oracle-geometry baseline on the new test protocol, and cite/disclose the prior study.

**Frozen boundary:** Paper I assumes true, registered obstacle geometry and DOES NOT contain vision. Paper II incorporates sensor-derived observability and uncertainty; it DOES NOT retroactively claim that Paper I had vision.

## 7. Decision gates

- P1-G0: read-only claim audit and scope freeze; no changes to legacy checkpoints.
- P1-G1: confirm old results under versioned configs, independent uncensored detector, matched strong QP / planner.
- P1-G2: conditional stop-tail + nonlinear recheck implementation and mechanism ablation. If not beneficial, retain measured audit paper and downgrade method claim.
- P1-G3: final paper rebuild from generated tables; submit only after citation/source check.
- P2-G0: Paper-I control API and evaluator stabilized; sensor-emulator benchmark designed.
- P2-G1: calibrated partial-geometry representation improves uncertainty fidelity.
- P2-G2: risk-aware shield outperforms matched worst-case fixed-margin and naive gating at comparable risk.
- P2-G3: trained policy improves the risk–completion frontier without exploiting censoring/unknown space.
- P2-G4: journal-grade OOD replication, robustness, safety claims qualified, physical feasibility evidence if available.

**Prohibition:** Agent orchestration, LLM RL, VLA, full-grape semantic recognition, and unvalidated diffusion augmentation are NOT prerequisites for either paper.
