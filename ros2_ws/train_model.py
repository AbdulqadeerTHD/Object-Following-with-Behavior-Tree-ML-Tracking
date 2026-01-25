#!/usr/bin/env python3
"""
YOLOv8 Model Training Script

This script trains YOLOv8 models for person and obstacle detection.
It supports three training stages:
1. Person detection model (person_obstacle_detector)
2. Obstacle detection model (obstacle_detector) - fine-tuned from person model
3. Combined model (person_obstacle_detector2) - person + obstacles together

Usage:
    python3 train_model.py --stage 1  # Train person detection model
    python3 train_model.py --stage 2  # Fine-tune for obstacles
    python3 train_model.py --stage 3  # Train combined model
"""

import os
import sys
import argparse
from pathlib import Path
from ultralytics import YOLO


def train_person_detector(workspace_root):
    """
    Stage 1: Train person detection model from scratch
    
    Uses yolo_dataset with person class only
    """
    print("=" * 60)
    print("Stage 1: Training Person Detection Model")
    print("=" * 60)
    
    # Paths
    model_path = "yolov8n.pt"  # Start from pre-trained YOLOv8n
    data_yaml = os.path.join(workspace_root, "training_data/yolo_dataset/dataset.yaml")
    project_dir = os.path.join(workspace_root, "runs/detect")
    name = "person_obstacle_detector"
    
    # Check if dataset exists
    if not os.path.exists(data_yaml):
        print(f"ERROR: Dataset YAML not found: {data_yaml}")
        return False
    
    print(f"Model: {model_path}")
    print(f"Dataset: {data_yaml}")
    print(f"Output: {project_dir}/{name}")
    print()
    
    # Load model
    model = YOLO(model_path)
    
    # Train
    results = model.train(
        data=data_yaml,
        epochs=100,
        imgsz=640,
        batch=8,
        device='cpu',  # Change to 'cuda' if GPU available
        workers=8,
        project=project_dir,
        name=name,
        patience=100,
        save=True,
        verbose=True,
        seed=0,
        deterministic=True,
        # Augmentation
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=0.0,
        translate=0.1,
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
        auto_augment='randaugment',
        erasing=0.4,
    )
    
    print(f"\nTraining completed! Model saved to: {project_dir}/{name}/weights/best.pt")
    return True


def train_obstacle_detector(workspace_root):
    """
    Stage 2: Fine-tune person model for obstacle detection
    
    Uses obstacles_dataset with obstacle class
    Fine-tunes from person_obstacle_detector model
    """
    print("=" * 60)
    print("Stage 2: Fine-tuning for Obstacle Detection")
    print("=" * 60)
    
    # Paths
    base_model = os.path.join(workspace_root, "runs/detect/person_obstacle_detector/weights/best.pt")
    data_yaml = os.path.join(workspace_root, "training_data/obstacles_dataset/dataset.yaml")
    project_dir = os.path.join(workspace_root, "runs/detect")
    name = "obstacle_detector"
    
    # Check if base model exists
    if not os.path.exists(base_model):
        print(f"ERROR: Base model not found: {base_model}")
        print("Please run Stage 1 first (train person detector)")
        return False
    
    # Check if dataset exists
    if not os.path.exists(data_yaml):
        print(f"ERROR: Dataset YAML not found: {data_yaml}")
        return False
    
    print(f"Base Model: {base_model}")
    print(f"Dataset: {data_yaml}")
    print(f"Output: {project_dir}/{name}")
    print()
    
    # Load model
    model = YOLO(base_model)
    
    # Fine-tune
    results = model.train(
        data=data_yaml,
        epochs=50,
        imgsz=640,
        batch=16,
        device='cpu',  # Change to 'cuda' if GPU available
        workers=8,
        project=project_dir,
        name=name,
        patience=10,
        save=True,
        verbose=True,
        seed=0,
        deterministic=True,
        # Augmentation
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=0.0,
        translate=0.1,
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
        auto_augment='randaugment',
        erasing=0.4,
    )
    
    print(f"\nFine-tuning completed! Model saved to: {project_dir}/{name}/weights/best.pt")
    return True


def train_combined_model(workspace_root):
    """
    Stage 3: Train combined model (person + obstacles)
    
    Uses combined_dataset with both person and obstacle classes
    Fine-tunes from person_obstacle_detector model
    """
    print("=" * 60)
    print("Stage 3: Training Combined Model (Person + Obstacles)")
    print("=" * 60)
    
    # Paths
    base_model = os.path.join(workspace_root, "runs/detect/person_obstacle_detector/weights/best.pt")
    data_yaml = os.path.join(workspace_root, "training_data/combined_dataset/dataset.yaml")
    project_dir = os.path.join(workspace_root, "runs/detect")
    name = "person_obstacle_detector2"
    
    # Check if base model exists
    if not os.path.exists(base_model):
        print(f"ERROR: Base model not found: {base_model}")
        print("Please run Stage 1 first (train person detector)")
        return False
    
    # Check if dataset exists
    if not os.path.exists(data_yaml):
        print(f"ERROR: Dataset YAML not found: {data_yaml}")
        return False
    
    print(f"Base Model: {base_model}")
    print(f"Dataset: {data_yaml}")
    print(f"Output: {project_dir}/{name}")
    print()
    
    # Load model
    model = YOLO(base_model)
    
    # Train combined model
    results = model.train(
        data=data_yaml,
        epochs=100,
        imgsz=640,
        batch=16,
        device='cpu',  # Change to 'cuda' if GPU available
        workers=8,
        project=project_dir,
        name=name,
        patience=15,
        save=True,
        verbose=True,
        seed=0,
        deterministic=True,
        # Augmentation
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=0.0,
        translate=0.1,
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
        auto_augment='randaugment',
        erasing=0.4,
    )
    
    print(f"\nTraining completed! Model saved to: {project_dir}/{name}/weights/best.pt")
    return True


def main():
    parser = argparse.ArgumentParser(description='Train YOLOv8 models for person and obstacle detection')
    parser.add_argument('--stage', type=int, choices=[1, 2, 3], required=True,
                        help='Training stage: 1=person, 2=obstacles, 3=combined')
    parser.add_argument('--workspace', type=str, default=None,
                        help='Path to workspace root (default: current directory)')
    
    args = parser.parse_args()
    
    # Determine workspace root
    if args.workspace:
        workspace_root = os.path.abspath(args.workspace)
    else:
        # Assume script is in ros2_ws directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = script_dir
    
    print(f"Workspace root: {workspace_root}")
    print()
    
    # Run appropriate training stage
    success = False
    if args.stage == 1:
        success = train_person_detector(workspace_root)
    elif args.stage == 2:
        success = train_obstacle_detector(workspace_root)
    elif args.stage == 3:
        success = train_combined_model(workspace_root)
    
    if success:
        print("\n" + "=" * 60)
        print("Training completed successfully!")
        print("=" * 60)
        sys.exit(0)
    else:
        print("\n" + "=" * 60)
        print("Training failed!")
        print("=" * 60)
        sys.exit(1)


if __name__ == '__main__':
    main()

