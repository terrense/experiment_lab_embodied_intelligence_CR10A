@echo off
REM Experience 5: same as 4, with the dynamics identified on the real CR10A
chcp 65001 >nul
echo Experience 5: full pipeline with the dynamics identified on the real CR10A (mass x1.64, real friction)
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash hands_on/4_moveit_rviz_mujoco.sh identified_run02 3
echo.
echo Done. Press any key to close.
pause >nul
