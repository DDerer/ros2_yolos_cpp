# OBBDetection2D.msg
# A single oriented bounding box detection result

# Timestamp and frame
std_msgs/Header header

# The oriented bounding box
ros2_yolos_cpp/OrientedBoundingBox2D bbox

# Class ID
int32 class_id

# Class name (optional)
string class_name

# Confidence score (0.0 - 1.0)
float64 score
