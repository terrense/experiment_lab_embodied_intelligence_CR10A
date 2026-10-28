@echo off
REM Experience 2: straight-line Jacobian controller (DLS) - watch it hit the wall (goal turns red)
chcp 65001 >nul
echo Experience 2: straight-line Jacobian controller (DLS) - watch it hit the wall (goal turns red)
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && cd learning && python3 -m ex_learning.view_policy --task wall --policy dls"
echo.
echo Done. Press any key to close.
pause >nul
