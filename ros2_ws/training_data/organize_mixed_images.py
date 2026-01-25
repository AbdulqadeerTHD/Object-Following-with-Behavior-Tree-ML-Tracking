#!/usr/bin/env python3
"""
Organize images that contain multiple classes
For images with both person and obstacles, put in person/ folder
"""

import sys
import os
import shutil
from pathlib import Path

def organize_mixed_image(image_path, has_person=True, has_obstacles=False):
    base_dir = Path(__file__).parent
    
    if has_person:
        target_dir = base_dir / "person"
    elif has_obstacles:
        target_dir = base_dir / "obstacles"
    else:
        target_dir = base_dir / "background"
    
    target_dir.mkdir(parents=True, exist_ok=True)
    
    source_path = Path(image_path)
    if not source_path.exists():
        print(f"Error: Image not found: {image_path}")
        return False
    
    filename = source_path.name
    counter = 1
    target_path = target_dir / filename
    
    while target_path.exists():
        stem = source_path.stem
        suffix = source_path.suffix
        target_path = target_dir / f"{stem}_{counter}{suffix}"
        counter += 1
    
    shutil.copy2(source_path, target_path)
    
    print(f"Copied to: {target_path}")
    if has_person and has_obstacles:
        print("NOTE: This image contains BOTH person and obstacles.")
        print("      Remember to label BOTH in the annotation file!")
        print("      Create: {}.txt with both class 0 (person) and class 1 (obstacles)".format(target_path.stem))
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 organize_mixed_images.py <image> person")
        print("  python3 organize_mixed_images.py <image> person+obstacles")
        print("  python3 organize_mixed_images.py <image> obstacles")
        print("  python3 organize_mixed_images.py <image> background")
        sys.exit(1)
    
    image_path = sys.argv[1]
    category = sys.argv[2] if len(sys.argv) > 2 else "person"
    
    if category == "person+obstacles" or category == "both":
        organize_mixed_image(image_path, has_person=True, has_obstacles=True)
    elif category == "person":
        organize_mixed_image(image_path, has_person=True, has_obstacles=False)
    elif category == "obstacles":
        organize_mixed_image(image_path, has_person=False, has_obstacles=True)
    elif category == "background":
        organize_mixed_image(image_path, has_person=False, has_obstacles=False)
    else:
        print(f"Unknown category: {category}")
        sys.exit(1)



