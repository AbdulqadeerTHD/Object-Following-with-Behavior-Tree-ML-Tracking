#!/usr/bin/env python3
"""
Dataset Preparation Script

Splits collected images into train/val (80/20) and creates label directories.

Usage:
    python3 prepare_dataset.py --images dataset/images --output dataset
"""

import argparse
import shutil
from pathlib import Path
import random

def split_dataset(images_dir, output_dir, train_ratio=0.8):
    """Split images into train and validation sets."""
    images_path = Path(images_dir)
    output_path = Path(output_dir)
    
    # Create directories
    train_img_dir = output_path / 'images' / 'train'
    val_img_dir = output_path / 'images' / 'val'
    train_label_dir = output_path / 'labels' / 'train'
    val_label_dir = output_path / 'labels' / 'val'
    
    for dir_path in [train_img_dir, val_img_dir, train_label_dir, val_label_dir]:
        dir_path.mkdir(parents=True, exist_ok=True)
    
    # Get all image files
    image_files = sorted(list(images_path.glob('*.jpg')) + list(images_path.glob('*.png')))
    
    if not image_files:
        print(f"❌ No images found in {images_path}")
        return
    
    # Shuffle and split
    random.seed(42)  # For reproducibility
    random.shuffle(image_files)
    
    split_idx = int(len(image_files) * train_ratio)
    train_images = image_files[:split_idx]
    val_images = image_files[split_idx:]
    
    print(f"Found {len(image_files)} images")
    print(f"Train: {len(train_images)} images")
    print(f"Val: {len(val_images)} images")
    
    # Copy images
    print("\nCopying images...")
    for img in train_images:
        shutil.copy(img, train_img_dir / img.name)
    for img in val_images:
        shutil.copy(img, val_img_dir / img.name)
    
    # Create placeholder label files
    print("\nCreating placeholder label files...")
    print("⚠️  IMPORTANT: You must label these images using LabelImg!")
    print(f"   Label directory: {output_path / 'labels'}")
    
    for img in train_images:
        label_file = train_label_dir / (img.stem + '.txt')
        if not label_file.exists():
            label_file.touch()  # Create empty file
    
    for img in val_images:
        label_file = val_label_dir / (img.stem + '.txt')
        if not label_file.exists():
            label_file.touch()  # Create empty file
    
    # Create data.yaml
    data_yaml = output_path / 'data.yaml'
    yaml_content = f"""nc: 1
names: ["person"]
path: {output_path.absolute()}
train: images/train
val: images/val
"""
    with open(data_yaml, 'w') as f:
        f.write(yaml_content)
    
    print(f"\n✅ Dataset prepared at: {output_path}")
    print(f"✅ Created data.yaml")
    print(f"\nNext steps:")
    print(f"1. Install LabelImg: pip3 install labelImg")
    print(f"2. Open LabelImg: labelImg")
    print(f"3. Open directory: {train_img_dir}")
    print(f"4. Save labels to: {train_label_dir}")
    print(f"5. Label ONLY persons (class 0)")
    print(f"6. Repeat for validation set")
    print(f"7. Train: python3 train_yolov8.py --data {data_yaml}")

def main():
    parser = argparse.ArgumentParser(description='Prepare dataset for YOLOv8 training')
    parser.add_argument('--images', type=str, default='dataset/images',
                        help='Directory containing collected images')
    parser.add_argument('--output', type=str, default='dataset',
                        help='Output directory for prepared dataset')
    parser.add_argument('--train-ratio', type=float, default=0.8,
                        help='Train/validation split ratio (default: 0.8)')
    
    args = parser.parse_args()
    
    split_dataset(args.images, args.output, args.train_ratio)

if __name__ == '__main__':
    main()



