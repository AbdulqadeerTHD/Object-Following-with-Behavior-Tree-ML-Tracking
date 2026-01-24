# Teleop Command Automation Guide

## Overview
This guide shows how to record your teleop commands from Webots console and replay them automatically, making the robot move autonomously with your exact motions.

## Step 1: Record Your Teleop Commands

### Terminal 1: Launch the System
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_moving_world.launch.py
```

### Terminal 2: Start Teleop Recorder
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 run object_following_control teleop_recorder --mode record --output my_motions.json
```

### Terminal 3: Run Teleop (Your Commands Will Be Recorded)
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Now move the robot using teleop:**
- Use `i` (forward), `,` (backward), `j` (left), `l` (right)
- All your commands will be recorded to `my_motions.json`
- When done, press **Ctrl+C** in Terminal 2 to stop recording

**The recorded file will be saved at:**
```
~/Documents/Path_Following_Robot/ros2_ws/my_motions.json
```

## Step 2: Replay Your Commands Automatically

### Terminal 1: Launch the System (Same as before)
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_moving_world.launch.py
```

### Terminal 2: Replay Your Recorded Commands
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 run object_following_control teleop_recorder --mode replay --input my_motions.json --replay_speed 1.0
```

**The robot will now execute your exact motions automatically!**

## Replay Speed Options

- `--replay_speed 1.0` = Normal speed (same as you recorded)
- `--replay_speed 0.5` = Half speed (slower)
- `--replay_speed 2.0` = Double speed (faster)

## Example Workflow

1. **Record your exploration path:**
   ```bash
   # Terminal 1: Launch
   ros2 launch object_following_integration launch_moving_world.launch.py
   
   # Terminal 2: Record
   ros2 run object_following_control teleop_recorder --mode record --output exploration.json
   
   # Terminal 3: Teleop (move robot around)
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   # ... move robot with keyboard ...
   # Press Ctrl+C in Terminal 2 when done
   ```

2. **Replay the exact same path:**
   ```bash
   # Terminal 1: Launch
   ros2 launch object_following_integration launch_moving_world.launch.py
   
   # Terminal 2: Replay
   ros2 run object_following_control teleop_recorder --mode replay --input exploration.json
   ```

## File Format

The recorded file (`my_motions.json`) contains:
```json
{
  "total_commands": 150,
  "duration": 45.2,
  "commands": [
    {
      "time": 0.0,
      "linear_x": 0.15,
      "angular_z": 0.0
    },
    {
      "time": 0.1,
      "linear_x": 0.15,
      "angular_z": 0.0
    },
    ...
  ]
}
```

## Tips

1. **Move slowly** when recording - this gives more precise control
2. **Record complete paths** - include full exploration sequences
3. **Test replay** - Make sure the replay works before relying on it
4. **Multiple recordings** - You can create multiple JSON files for different paths

## Troubleshooting

**Problem**: Replay doesn't work
- **Solution**: Make sure the launch file is running and `/cmd_vel` topic is available

**Problem**: Robot moves too fast/slow
- **Solution**: Adjust `--replay_speed` parameter

**Problem**: Commands not recorded
- **Solution**: Make sure teleop is publishing to `/cmd_vel` and recorder is subscribed


