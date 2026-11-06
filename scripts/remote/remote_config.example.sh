#!/usr/bin/env bash
# Remote compute configuration for the H20 GPU server.
#
# COPY THIS FILE to remote_config.local.sh and fill in the values. The copy is git-ignored:
# no host name, no user name and no path of the owner's infrastructure goes into the public
# repository.
#
#   cp scripts/remote/remote_config.example.sh scripts/remote/remote_config.local.sh
#
# Every script in scripts/remote/ sources remote_config.local.sh if it exists.

# The SSH target. The key and the port are whatever the owner already uses; nothing is assumed.
REMOTE_HOST=""                 # e.g. user@server.example.com
REMOTE_PORT=""                 # optional, e.g. 22
REMOTE_DIR=""                  # absolute path of the checkout on the server

# The python interpreter on the server (a venv is created by setup_env.sh).
REMOTE_PY=""

# How many training jobs to run in parallel. The MuJoCo plant is CPU-bound, so the useful
# number is roughly (server cores) / 13 (12 envs + the training process). The setup script
# prints a recommendation.
REMOTE_JOBS=""

# The seeds to train on the server. The seeds already trained locally are NOT repeated.
REMOTE_SEEDS="9 10 11 12 13 14 15 16 17 18 19 20"

# Training budget, matching the archived policies exactly (5M timesteps, 12 envs).
REMOTE_TIMESTEPS="5000000"
REMOTE_N_ENVS="12"
REMOTE_TASK="wall_rand"
