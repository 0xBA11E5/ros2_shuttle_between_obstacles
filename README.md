# ShuttleLogic Node README

## Overview
ros2 package that lets robot follow line and turn when obstacles are blocking the path.
## Dependencies
- ROS2 (Humble)
- `cv2` (OpenCV) - Used for image processing (although not currently used in the provided code)
- `numpy` - General-purpose array processing library (although not currently used in the provided code)
- `cv_bridge` - Bridge to convert ROS2 image messages into OpenCV images
- ROS2 message types: `std_msgs.msg.String`, `geometry_msgs.msg.Twist`, `sensor_msgs.msg.CompressedImage`

## Parameters
The following parameters can be declared to adjust the robot's behavior:
- `FollowLineSpeed`: Linear speed when following a line.
- `FollowLineTurn`: Angular speed when following a line.
- `FollowObjectSpeed`: Linear speed when getting close to an object.
- `FollowObjectTurn`: Angular speed when following an object. (not used)

## Topics
- **InputStateMachineFollowLine**: A subscription to receive commands for line-following behavior. Expected message type: `Twist`.
- **InputStateMachineFollowObject**: A subscription to receive commands for object-following behavior. Expected message type: `Twist`.
- **cmd_vel**: A publisher that sends the robot’s velocity commands. Expected message type: `Twist`.

## Functionality
1. **Line Following**: When the robot is instructed to follow a line, the node reads the speed and turning rate from the `InputStateMachineFollowLine` topic and drives the robot accordingly.
2. **Obstacle Avoidance**: When the robot detects an obstacle (i.e., `FollowObjectSpeed` is 0), it stops and turns by a specified angle to avoid the obstacle. The robot will then attempt to follow the object again once the obstacle is avoided.
3. **TimerCallback**: A periodic callback that publishes the appropriate movement commands based on the current behavior (line following or object avoidance).

## How to Run
1. Make sure that ROS2 and the required dependencies are installed.
2. Compile the package containing this node.
3. Launch the ROS2 system and run the node:
   ```bash
   ros2 launch shuttle_between_obstacles shuttle_between_obstacles_launch.py
