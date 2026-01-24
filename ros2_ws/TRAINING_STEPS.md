# Step-by-Step Training Guide

## Step 1: Data Collection (Manual Teleop)

### Terminal 1: Launch Webots
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_moving_world.launch.py \
    target_class_name:=person \
    confidence_threshold:=0.2
```

### Terminal 2: Start Data Collector
```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 run object_following_vision data_collector
```

### Terminal 3: Manual Teleop Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffdrive_controller/cmd_vel_unstamped
```

**Instructions:**
- Move robot **slowly** using teleop
- Cover different areas: inside, outside, corners
- Get different distances from persons
- Get different angles
- Collect 300-600 images minimum

**Teleop Controls:**
- `i` = Forward
- `,` = Backward
- `j` = Turn left
- `l` = Turn right
- `k` = Stop

**Check progress:**
```bash
ls -la ~/Documents/Path_Following_Robot/ros2_ws/dataset/images/ | wc -l
```

**Stop when you have 300-600 images** (Ctrl+C in Terminal 2)

---

## Step 2: Prepare Dataset

```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
python3 prepare_dataset.py
```

This will:
- Split images into train/val (80/20)
- Create label directories
- Create data.yaml

---

## Step 3: Label Images

### Install LabelImg
```bash
pip3 install labelImg
```

### Open LabelImg
```bash
labelImg
```

### Label Training Images
1. Click "Open Dir" → Select: `~/Documents/Path_Following_Robot/ros2_ws/dataset/images/train/`
2. Click "Change Save Dir" → Select: `~/Documents/Path_Following_Robot/ros2_ws/dataset/labels/train/`
3. Make sure **"YOLO"** format is selected (not PascalVOC)
4. For each image:
   - Draw bounding box **ONLY around person**
   - Press `W` to create box
   - Press `Ctrl+S` to save
   - Press `D` for next image
5. **Ignore**: boxes, chairs, tables, walls (only label persons!)

### Label Validation Images
1. Click "Open Dir" → Select: `~/Documents/Path_Following_Robot/ros2_ws/dataset/images/val/`
2. Click "Change Save Dir" → Select: `~/Documents/Path_Following_Robot/ros2_ws/dataset/labels/val/`
3. Repeat labeling process

---

## Step 4: Train YOLOv8

```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
python3 train_yolov8.py --data dataset/data.yaml --epochs 50 --batch 16
```

**Training will:**
- Train for 50 epochs
- Save best model as `best.pt`
- Show training progress

**Time:** 30-60 minutes depending on GPU

---

## Step 5: Use Trained Model

After training, use the model:

```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_moving_world.launch.py \
    target_class_name:=person \
    confidence_threshold:=0.2 \
    --ros-args \
    -p model_path:=best.pt
```

---

## Quick Reference

```bash
# 1. Collect images (3 terminals)
# Terminal 1: Launch Webots
ros2 launch object_following_integration launch_moving_world.launch.py target_class_name:=person confidence_threshold:=0.2

# Terminal 2: Data collector
ros2 run object_following_vision data_collector

# Terminal 3: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffdrive_controller/cmd_vel_unstamped

# 2. Prepare dataset
python3 prepare_dataset.py

# 3. Label images (LabelImg)
labelImg

# 4. Train
python3 train_yolov8.py --epochs 50

# 5. Use trained model
ros2 launch object_following_integration launch_moving_world.launch.py \
    target_class_name:=person confidence_threshold:=0.2 \
    --ros-args -p model_path:=best.pt
```



