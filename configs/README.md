# configs

Run-mode configuration lives inside the ROS package so it is installed with it:

| Mode | File |
|---|---|
| SIMULATION | `ros2_ws/src/ex_bringup/config/modes/simulation.yaml` |
| REPLAY | `ros2_ws/src/ex_bringup/config/modes/replay.yaml` |
| LIVE_OBSERVE | `ros2_ws/src/ex_bringup/config/modes/live_observe.yaml` (`enable_live_connection: false` by default) |

Frame conventions: `ros2_ws/src/ex_description/config/cr10_frames.yaml`.
