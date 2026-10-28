@echo off
REM Experience 3: the trained PPO policy (seed 2, best of 3 seeds: 80 pct success)
chcp 65001 >nul
echo Experience 3: the trained PPO policy (seed 2, best of 3 seeds: 80 pct success)
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && cd learning && ~/ex_rlenv/bin/python -m ex_learning.view_policy --task wall --policy ppo --model ../data/reports/learning/ppo_wall_s2"
echo.
echo Done. Press any key to close.
pause >nul
