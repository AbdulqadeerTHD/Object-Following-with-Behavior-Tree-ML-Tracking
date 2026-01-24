# Data Collection Guide for YOLOv8 Training

## Current Setup
- **Data Collector**: Running automatically, saves every 5 frames
- **Max Images**: 600 images
- **Output**: `ros2_ws/dataset/images/`
- **Teleop Controls**: Use keyboard to move robot

## Recommended Movement Patterns

### Pattern 1: Circular Approach (Best for diverse angles)
1. Start far from the person
2. Move in a circle around them (360° rotation)
3. Gradually get closer while circling
4. Then move away while circling
5. Repeat from different starting positions

### Pattern 2: Grid Pattern
1. Move in a grid pattern around the person
2. Stop at each grid point
3. Rotate robot 360° at each stop
4. Capture person from all angles at each distance

### Pattern 3: Random Exploration
1. Move randomly through the environment
2. When you see the person, approach from different angles
3. Rotate around them
4. Move away and come back from different directions

## Key Movements to Include

### ✅ DO:
- **Slow movements** - Let data collector capture more frames
- **360° rotations** - Capture person from all angles
- **Different distances** - Close (1-2m), Medium (3-5m), Far (5-10m)
- **Different angles** - Front, side, back, diagonal views
- **Arcs and curves** - Not just straight lines
- **Stop and rotate** - Pause at different positions and rotate
- **Partial views** - Person partially visible (behind obstacles, edge of frame)

### ❌ DON'T:
- Don't move too fast (collector might miss frames)
- Don't only capture from one angle
- Don't only stay at one distance
- Don't ignore background variety

## Teleop Controls Reference

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i = forward
, = backward
j = turn left
l = turn right
k = stop

w/x = increase/decrease linear speed
e/c = increase/decrease angular speed
```

## Collection Strategy

### Phase 1: Wide Coverage (First 200 images)
- Move around entire environment
- Capture person from far distances
- Include different backgrounds

### Phase 2: Close-up Views (Next 200 images)
- Approach person closely
- Rotate around them at close range
- Capture detailed views

### Phase 3: Mixed Angles (Final 200 images)
- Mix of distances and angles
- Include edge cases (partial views, different lighting)
- Fill any gaps in coverage

## Monitoring Collection

Check progress in data collector terminal:
- It logs every 50 images saved
- Watch for: `Saved 50 images...`, `Saved 100 images...`, etc.
- When you reach 600 images, it will stop automatically

## After Collection

1. Check collected images:
   ```bash
   ls -lh ros2_ws/dataset/images/ | wc -l
   ```

2. Review a few images to ensure quality:
   ```bash
   # View images (if you have an image viewer)
   # Make sure person is visible in most images
   ```

3. Follow `TRAINING_STEPS.md` for next steps (labeling and training)

## Tips

- **Take your time** - Quality over quantity
- **Move slowly** - Better image variety
- **Check Webots view** - Make sure person is in camera frame
- **Vary everything** - Distance, angle, background, lighting
- **Stop periodically** - Rotate 360° at different positions


