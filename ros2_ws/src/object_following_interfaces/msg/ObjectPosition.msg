# Custom message for object position tracking
# Per specification: found (bool), x (0-1 normalized), y (0-1 normalized), distance (optional)

std_msgs/Header header
bool found                    # Whether the target object is currently detected
float32 x                     # Normalized x-center of bounding box [0.0, 1.0]
float32 y                     # Normalized y-center of bounding box [0.0, 1.0]
float32 distance              # Optional distance estimate (based on bounding box size)
