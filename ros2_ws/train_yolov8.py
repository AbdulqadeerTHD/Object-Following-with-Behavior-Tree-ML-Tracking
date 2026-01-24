#!/usr/bin/env python3
"""
YOLOv8 Training Script for Webots Person Detection

Trains YOLOv8 on images collected from Webots camera.

Usage:
    python3 train_yolov8.py --data dataset/data.yaml --epochs 50
"""

import argparse
from pathlib import Path
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description='Train YOLOv8 for person detection')
    parser.add_argument('--data', type=str, default='dataset/data.yaml',
                        help='Path to data.yaml file')
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                        help='Base model (yolov8n.pt, yolov8s.pt, etc.)')
    parser.add_argument('--epochs', type=int, default=50,
                        help='Number of training epochs')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Image size for training')
    parser.add_argument('--batch', type=int, default=16,
                        help='Batch size')
    parser.add_argument('--device', type=str, default='',
                        help='Device (cuda, cpu, or leave empty for auto)')
    
    args = parser.parse_args()
    
    data_yaml = Path(args.data)
    if not data_yaml.exists():
        print(f"❌ Error: data.yaml not found at {data_yaml}")
        print(f"   Run prepare_dataset.py first!")
        return
    
    print(f"Starting YOLOv8 training...")
    print(f"Data config: {data_yaml}")
    print(f"Base model: {args.model}")
    print(f"Epochs: {args.epochs}")
    print(f"Image size: {args.imgsz}")
    print(f"Batch size: {args.batch}")
    
    # Load model
    model = YOLO(args.model)
    
    # Train
    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project='runs/detect',
        name='person_detection',
        exist_ok=True,
        pretrained=True,
        optimizer='AdamW',
        lr0=0.01,
        patience=10,
        save=True,
        plots=True
    )
    
    # Save best model as best.pt
    best_model_path = Path('runs/detect/person_detection/weights/best.pt')
    if best_model_path.exists():
        import shutil
        shutil.copy(best_model_path, 'best.pt')
        print(f"\n✅ Training complete!")
        print(f"Best model saved as: best.pt")
        print(f"Use this model in launch file with: model_path:=best.pt")
    else:
        print(f"\n⚠️  Warning: best.pt not found at expected location")
        print(f"Check: runs/detect/person_detection/weights/")

if __name__ == '__main__':
    main()



