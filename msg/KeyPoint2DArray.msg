# KeyPoint2DArray.msg
# Array of 2D keypoints for a single person/object

# Header with timestamp
std_msgs/Header header

# Array of keypoints (17 for COCO format)
ros2_yolos_cpp/KeyPoint2D[] keypoints

# Bounding box of the detected person (optional)
geometry_msgs/Point bounding_box_center
float64 bounding_box_size_x
float64 bounding_box_size_y

# Detection confidence
float64 confidence

# Class ID (typically 0 for person in pose estimation)
int32 class_id
