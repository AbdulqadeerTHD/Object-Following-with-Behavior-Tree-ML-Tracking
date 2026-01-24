# 🤖 TRAINING MODE: Manual Control + Autonomous Behavior

This setup allows you to:
1. **Manually move the robot** through the environment with keyboard
2. **System automatically captures images** from the camera
3. **Robot learns** to detect pedestrians/persons
4. **Robot autonomously** searches, detects, approaches, and stops near pedestrians

## Fresh Training Environment

**File**: `training_world.wbt`
- Clean 20x20m environment
- 3 pedestrians at different locations
- 3 maze blocks for obstacles
- Full LiDAR with blue visualization
- Pedestrians remain STATIONARY (good for training)

## How It Works

### What YOU do:
- **Manually move the robot** around the environment with keyboard
- Robot visits different areas and approaches pedestrians

### What the SYSTEM does:
- **Automatically captures images** every time robot moves
- **Detects pedestrians** using YOLOv8
- **Moves autonomously** toward detected pedestrians
- **Saves images** to `dataset/images/` for training
- **Records motion** for playback

## Run Instructions

### Terminal 1: Launch Training System
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch object_following_integration launch_training.launch.py
```

**What starts**:
- ✅ Webots with training_world.wbt
- ✅ Data collector (automatically saving images)
- ✅ Vision node (detecting pedestrians)
- ✅ Behavior tree (state machine)
- ✅ Control node (movement)

### Terminal 2: Manual Teleop (YOU move the robot)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls**:
- Press 'q' to increase speed (10+ times for max speed)
- Press 'i' to move FORWARD
- Press 'j' to turn LEFT
- Press 'l' to turn RIGHT
- Press 'k' to STOP
- Hold 'i' to move continuously

### What Happens During Training:

1. **SEARCH MODE**: Robot rotates looking for pedestrians
2. **DETECT**: When camera detects a person, robot locks on
3. **APPROACH**: Robot moves toward the pedestrian
4. **STOP**: Stops at safe distance (0.5m)
5. **IMAGES SAVED**: Every moment is captured in `dataset/images/`

### After Training Session:

Check images saved:
```bash
ls -la ~/Documents/Path_Following_Robot/ros2_ws/dataset/images/ | head -20
```

Count total images:
```bash
ls ~/Documents/Path_Following_Robot/ros2_ws/dataset/images/ | wc -l
```

## System Features

✅ **Automatic Image Collection**: Saves whenever something changes
✅ **Person Detection**: YOLOv8 detects pedestrians in real-time  
✅ **Autonomous Behavior**: 
   - SEARCH: Look for people
   - DETECT: Found someone!
   - APPROACH: Move toward them
   - STOP: Safe distance reached
   - SEARCH: Look for next person

✅ **LiDAR**: Blue point cloud visible, obstacle avoidance active
✅ **Max Speed**: 100 m/s linear, 100 rad/s angular
✅ **Camera**: 640x480, continuously capturing

## Expected Results

After 5-10 minutes of manual exploration:
- **100-500 images collected** (showing pedestrian detection)
- **Video of autonomous behavior** (approach/stop working)
- **Ready for YOLO training** on your custom dataset

## Next Steps After Training:

1. Prepare dataset (split train/val)
2. Train YOLOv8 on collected images
3. Test trained model on new environments
4. Deploy autonomous detection

---

**Enjoy training! The robot will automatically collect data while you explore.** 🚀

