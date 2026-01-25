# Model Training Guide

This guide explains how to train the YOLOv8 models used in the object following robot system.

## Training Pipeline Overview

The training process consists of three stages:

1. **Stage 1: Person Detection Model** (`person_obstacle_detector`)
   - Trains from scratch using `yolov8n.pt` as base
   - Dataset: `training_data/yolo_dataset/` (person class only)
   - Output: `runs/detect/person_obstacle_detector/weights/best.pt`

2. **Stage 2: Obstacle Detection Model** (`obstacle_detector`) - Optional
   - Fine-tunes from person detection model
   - Dataset: `training_data/obstacles_dataset/` (obstacle class only)
   - Output: `runs/detect/obstacle_detector/weights/best.pt`

3. **Stage 3: Combined Model** (`person_obstacle_detector2`)
   - Fine-tunes from person detection model
   - Dataset: `training_data/combined_dataset/` (person + obstacle classes)
   - Output: `runs/detect/person_obstacle_detector2/weights/best.pt`

## Prerequisites

```bash
pip install ultralytics
```

## Training Commands

### Stage 1: Train Person Detection Model

```bash
cd ros2_ws
python3 train_model.py --stage 1
```

**Parameters:**
- Epochs: 100
- Batch size: 8
- Image size: 640x640
- Base model: yolov8n.pt
- Dataset: `training_data/yolo_dataset/dataset.yaml`

### Stage 2: Fine-tune for Obstacles (Optional)

```bash
cd ros2_ws
python3 train_model.py --stage 2
```

**Parameters:**
- Epochs: 50
- Batch size: 16
- Image size: 640x640
- Base model: `person_obstacle_detector/weights/best.pt`
- Dataset: `training_data/obstacles_dataset/dataset.yaml`

### Stage 3: Train Combined Model

```bash
cd ros2_ws
python3 train_model.py --stage 3
```

**Parameters:**
- Epochs: 100
- Batch size: 16
- Image size: 640x640
- Base model: `person_obstacle_detector/weights/best.pt`
- Dataset: `training_data/combined_dataset/dataset.yaml`

## Dataset Structure

### YOLO Dataset Format

Each dataset folder should contain:
```
dataset_name/
├── dataset.yaml          # Dataset configuration
├── train/
│   ├── images/          # Training images
│   └── labels/          # YOLO format labels (.txt)
└── val/
    ├── images/          # Validation images
    └── labels/          # YOLO format labels (.txt)
```

### Dataset YAML Format

```yaml
path: /path/to/dataset
train: train/images
val: val/images

names:
  0: person      # Class 0: Person
  1: obstacle    # Class 1: Obstacle (if applicable)
```

### Label Format (YOLO)

Each `.txt` file contains one line per object:
```
class_id center_x center_y width height
```

All values are normalized (0.0 to 1.0).

## Training Data Organization

### Raw Images

Raw training images are stored in:
- `training_data/person/` - Person images
- `training_data/obstacles/` - Obstacle images
- `training_data/background/` - Background/empty scenes

### Organized Datasets

Pre-organized datasets ready for training:
- `training_data/yolo_dataset/` - Person detection dataset
- `training_data/obstacles_dataset/` - Obstacle detection dataset
- `training_data/combined_dataset/` - Combined person + obstacle dataset

## Training Parameters

### Augmentation

The training uses the following augmentations:
- **HSV**: Hue ±0.015, Saturation ±0.7, Value ±0.4
- **Translation**: ±0.1
- **Scale**: ±0.5
- **Horizontal Flip**: 0.5 probability
- **Mosaic**: 1.0 (enabled)
- **Auto Augment**: RandAugment
- **Random Erasing**: 0.4 probability

### Hyperparameters

- **Learning Rate**: 0.01 (initial), 0.01 (final)
- **Momentum**: 0.937
- **Weight Decay**: 0.0005
- **Warmup Epochs**: 3
- **Loss Weights**: Box=7.5, Cls=0.5, DFL=1.5

## Model Usage

After training, the models are used in `vision_yolo_node.py`:

```python
# Current model path
model_path = "runs/detect/person_obstacle_detector/weights/best.pt"

# To use combined model (person + obstacles):
model_path = "runs/detect/person_obstacle_detector2/weights/best.pt"
```

## Training Results

Training results are saved in `runs/detect/<model_name>/`:
- `weights/best.pt` - Best model weights
- `weights/last.pt` - Last epoch weights
- `results.png` - Training curves
- `confusion_matrix.png` - Confusion matrix
- `args.yaml` - Training configuration

## Notes

- Training is done on CPU by default. Change `device='cpu'` to `device='cuda'` in `train_model.py` if GPU is available.
- Training time depends on dataset size and hardware. Person model (100 epochs) typically takes several hours on CPU.
- The current system uses `person_obstacle_detector` model (person class only).
- The `person_obstacle_detector2` model (combined) can be used if obstacle detection is needed.

