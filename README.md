# ShuttleLogic ROS2 Node

This ROS2 node implements basic logic for controlling a robot's movement based on two different inputs:
- **FollowLine**: The robot follows a line with a specified speed and turn rate.
- **FollowObject**: The robot follows nearby objects and stops infront of them

## Dependencies

This node requires the following packages:
- `rclpy` (ROS2 Python client library)
- `cv2` (OpenCV for handling image data)
- `numpy`
## default topics
- `sensor_msgs` (for image and sensor message types)
- `geometry_msgs` (for motion control messages)
- `cv_bridge` (for converting between ROS and OpenCV image formats)

launching:
```bash
ros2 launch shuttle_between_object shuttle_between_object
