@echo off
REM Stop every EMBODIED-X simulation window/process (MoveIt, RViz, MuJoCo, viewers). Does not touch training runs.
chcp 65001 >nul
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && bash scripts/m2_session.sh stop; pkill -f [e]x_learning.view_policy; echo all simulation windows closed"
echo Press any key to close.
pause >nul
