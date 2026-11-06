# Remote compute: the H20 server, and the rule for when to use it

**The rule, stated before any experiment is designed:** every experiment gets a compute
assessment first — *does this need more than the laptop?* — and the decision is written down in
`docs/PROGRESS.md` with the experiment. If the answer is "the laptop is enough", the experiment
runs on the laptop and the server is left alone.

The server is a plain Linux node with an NVIDIA H20. It is reached over SSH by the owner. **No
host name, user name or path of the owner's infrastructure is committed**: they live in
`scripts/remote/remote_config.local.sh`, which is git-ignored, and the committed file is
`remote_config.example.sh`.

---

## 1. The assessment

| Planned experiment | Laptop (16 cores, no GPU) enough? | What the server actually adds | Where it runs |
|---|---|---|---|
| **PPO seed training** (the M5 policies) | yes, but **37–41 min per seed, strictly serial** — the archived seeds are 5M timesteps on 12 subprocess envs | the MuJoCo plant is **CPU-bound**, so the win is the server's core count, not its GPU: many seeds in parallel | **server** |
| The safety filters (bounded QP, execution-aware) | yes — numpy + scipy, 0.3–2.5 ms per control step | nothing | laptop |
| The confirmatory benchmark / frontier sweeps | yes — a 45-cell run is ~2 h, dominated by the independent mesh checker (CPU) | nothing | laptop |
| The independent mesh checker | yes — numpy + scipy KD-trees | nothing | laptop |
| The statistics (paired / hierarchical bootstrap) | yes — milliseconds | nothing | laptop |
| **The residual-RL arm (B7, Week 5)** | marginal — SAC on CPU with 12 envs needs many more gradient updates than PPO | **CUDA for the network updates**, plus the same core count for the envs | server, later |
| A vision front-end / VLA policy | no | CUDA | **not planned** — out of scope for this question |

**The honest summary:** the H20's VRAM is not the binding constraint anywhere in the current
plan. What the server buys is *parallelism* — and parallelism is exactly what the study needs,
because the statistical weakness of the headline comparison is the number of training seeds
(three seeds cannot separate an 8-point mean difference from an 8-point between-seed spread).

---

## 2. Why the local machine could not simply train more seeds

The three archived policies took 2445 s, 2205 s and 2213 s (≈ 37–41 min each) for 5M timesteps
on 12 subprocess environments. Adding three more seeds locally is another ~2 h of wall clock, and
the laptop is also running the benchmark sweeps. Twelve more seeds — enough to make the
between-seed spread a usable estimate rather than an anecdote — is ~8 h locally and a fraction of
that on a server with many cores.

---

## 3. The procedure (what the owner does, once)

```bash
# 1. on the laptop (WSL or Git Bash): create the local config - the SSH target is NOT committed
cp scripts/remote/remote_config.example.sh scripts/remote/remote_config.local.sh
$EDITOR scripts/remote/remote_config.local.sh      # REMOTE_HOST, REMOTE_DIR

# 2. on the SERVER: create the environment (no ROS needed)
cd <checkout>
bash scripts/remote/setup_env.sh
#    -> prints the GPU model, the VRAM, the core count and the recommended parallelism

# 3. from the laptop: send the code and the plant description
bash scripts/remote/sync.sh

# 4. on the SERVER: train the seeds in parallel
bash scripts/remote/train_seeds.sh
#    -> REMOTE_SEEDS in the config; already-trained seeds are skipped

# 5. from the laptop: bring the policies back
bash scripts/remote/fetch_results.sh
```

The policies land in `data/reports/learning/ppo_wall_rand_s<seed>` exactly where a locally
trained one does, so every downstream script (the benchmark, the frontier sweep, the statistics)
works on them without knowing where they were trained.

---

## 4. What is deliberately NOT sent to the server

* **`ros2_ws/src/ex_dobot_motion`** — the only motion-capable package in the repository. A
  compute node must not hold a copy of the code that can move a robot. `sync.sh` excludes it, and
  the exclusion is deliberate rather than incidental.
* The trained policies that already exist locally (the server trains its own).
* `.git`, `build/`, `install/`, `log/`, the raw robot recordings and `third_party/`.

What IS sent is the minimum a simulation job needs: `learning/`, `evaluation/`, `scripts/`,
`ros2_ws/src/ex_description/` (the URDF and the meshes) and `data/cache/` (the **pre-expanded**
URDF, so the server needs neither ROS nor `xacro`).

---

## 5. Why the URDF is pre-expanded

`ex_learning.envs.cr10a_reach.expanded_urdf_text()` used to call `xacro` on every environment
construction — that is, on every episode of a sweep and every cell of a benchmark. It is now
**cached** in `data/cache/ex_cr10_calibrated_1.urdf` (7.8 kB, committed, and byte-identical to
what xacro produces). Two consequences: the expansion is 18× faster locally, and a remote node
needs no ROS installation at all.

---

## 6. The safety boundary on the server

The server runs **simulation only**. Nothing in the remote scripts opens a Dobot port, reads
robot state, or can move hardware; the motion-capable package is not even copied there. The
hardware stages of the research plan (the command-channel characterisation, the stop test, the
compliant trials) stay on the laptop with the owner present at the e-stop, exactly as the
standing rules require.
