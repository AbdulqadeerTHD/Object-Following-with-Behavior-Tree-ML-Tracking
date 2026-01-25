Training Data Organization

Folders:
- person/     : Images containing persons (pedestrians) - TARGET CLASS
- obstacles/  : Images containing obstacles (solid boxes, office chairs)
- background/ : Images with no target or obstacles (empty scenes)

How to organize screenshots:

1. Take screenshots from Webots simulation
2. Save them to this directory temporarily
3. Run the organize script:
   python3 organize_training_images.py <image_file> <category>

Example:
   python3 organize_training_images.py screenshot1.png person
   python3 organize_training_images.py screenshot2.png obstacles
   python3 organize_training_images.py screenshot3.png background

Or manually copy images to the appropriate folders.



