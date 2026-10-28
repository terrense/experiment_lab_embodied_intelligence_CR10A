@echo off
REM Experience 0: free-space reaching, no obstacles (DLS)
chcp 65001 >nul
echo Experience 0: free-space reaching, no obstacles (DLS)
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash -lc "source scripts/ex_env.sh && cd learning && python3 -m ex_learning.view_policy --task reach --policy dls"
echo.
echo Done. Press any key to close.
pause >nul
