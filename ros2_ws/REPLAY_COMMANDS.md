# Replay Your Recorded Commands

## Your Recorded File
- **File**: `teleop_commands.json`
- **Commands**: 51,187 commands
- **Duration**: ~30 minutes (1831 seconds)
- **Status**: ✅ Ready to replay

## How to Replay Your Commands

### Step 1: Launch the System
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_moving_world.launch.py
```

### Step 2: Replay Your Commands (Autonomous)
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 run object_following_control teleop_recorder --mode replay --input teleop_commands.json --replay_speed 1.0
```

**The robot will now execute your exact motions automatically!**

## Replay Speed Options

- `--replay_speed 1.0` = Normal speed (same as you recorded) - **Recommended**
- `--replay_speed 0.5` = Half speed (slower, more careful)
- `--replay_speed 2.0` = Double speed (faster)

## What Happens

1. Robot will execute all 51,187 commands in sequence
2. Takes ~30 minutes to complete (at 1.0x speed)
3. Robot moves exactly as you did during recording
4. Logs will show: "Replay complete!" when done

## Notes

- The timing has been normalized (starts from first command)
- All commands will be replayed in exact sequence
- Robot will move autonomously through your recorded path
- You can stop anytime with Ctrl+C

## Troubleshooting

**If replay doesn't start:**
- Make sure launch file is running first
- Check that `/cmd_vel` topic is available: `ros2 topic list | grep cmd_vel`

**If robot moves too fast/slow:**
- Adjust `--replay_speed` parameter
- Try 0.5 for slower, 2.0 for faster


