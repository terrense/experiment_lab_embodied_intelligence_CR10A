@echo off
REM Experience 4: MoveIt + RViz + MuJoCo physics, full pipeline
chcp 65001 >nul
echo Experience 4: MoveIt plans around the wall, RViz shows it, MuJoCo executes it with physics
wsl -d EmbodiedX-Humble -u ex --cd "%~dp0.." -- bash hands_on/4_moveit_rviz_mujoco.sh placeholder 3
echo.
echo Done. Press any key to close.
pause >nul
