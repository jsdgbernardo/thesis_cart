# thesis_cart

## Teleoperation (keyboard)

You can control the spawned robot with a keyboard teleop node that publishes geometry_msgs/Twist to `/diff_drive/cmd_vel` (the launch already bridges `/diff_drive/cmd_vel` → `/model/diff_drive/cmd_vel`).

- Run teleop in a separate terminal (recommended):

```bash
. install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive/cmd_vel
```

- Run teleop in a new GNOME Terminal:

```bash
gnome-terminal -- bash -lc ". install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive/cmd_vel; exec bash"
```

- Or in xterm:

```bash
xterm -e ". install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive/cmd_vel; bash"
```

Notes:
- If you launch teleop from within the same `ros2 launch` terminal, it may fail because `teleop_twist_keyboard` expects a TTY; using a separate terminal emulator avoids that.
- You can also enable the optional `teleop` launch argument to attempt to start the teleop command defined by `teleop_cmd` (default is `ros2 run teleop_twist_keyboard ...`). This may still require a terminal emulator wrapper for interactive input.
