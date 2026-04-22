# OrientedBoundingBox2D.msg
# Oriented (rotated) bounding box in 2D image space

# Center point of the bounding box
geometry_msgs/Point center

# Width and height of the box (before rotation)
float64 size_x
float64 size_y

# Rotation angle in radians (counter-clockwise from x-axis)
float64 theta
