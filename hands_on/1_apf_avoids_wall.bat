@echo off
REM Experience 1: potential-field controller (APF) reaches around the wall
chcp 65001 >nul
echo Experience 1: potential-field controller (APF) reaches around the wall
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && cd learning && python3 -m ex_learning.view_policy --task wall --policy apf"
echo.
echo Done. Press any key to close.
pause >nul
