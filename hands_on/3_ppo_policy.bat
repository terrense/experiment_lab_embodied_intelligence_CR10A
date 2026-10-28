@echo off
REM Experience 3: the trained PPO policy
chcp 65001 >nul
echo Experience 3: the trained PPO policy
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && cd learning && ~/ex_rlenv/bin/python -m ex_learning.view_policy --task wall --policy ppo --model ../data/reports/learning/ppo_wall_s0"
echo.
echo Done. Press any key to close.
pause >nul
