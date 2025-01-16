import rclpy
import rclpy.node
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class LineFollowing(rclpy.node.Node):

    def __init__(self):
        super().__init__('line_following')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 300)
        self.declare_parameter('boundary_right', 340)
        self.declare_parameter('threshold_line', 100)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 0.3)

        # init openCV-bridge
        self.bridge = CvBridge()

        # create subscribers for image data with changed qos
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'InputStateMachineFollowLine', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

        # variable for line position
        self.line_position = None


    # handling received image data
    def scanner_callback(self, data):

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]
        # get the lowest row from image (the line-following row)
        img_row = img_gray[height - 5, :]  # This is the last row (bottom row)
        #print(img_row)
        img_row[639] = 0 #somehow this is always close to the maximum value? meaning otherwise the robot will always steer to the right
        # Debugging: Check the shape of img_row
        #print(f"Image row shape: {img_row.shape}, Width: {width}")

        # find the index of the brightest pixel
        line_position = np.argmax(img_row)

        # Ensure the line_position is within bounds
        if line_position >= width:
            #print(f"Warning: line_position {line_position} exceeds image width {width}")
            line_position = width - 1  # Cap the value to avoid out-of-bounds access

        # store the line position
        self.line_position = line_position

        # for visualization (optional)
        cv2.imshow("IMG", img_gray)
        cv2.waitKey(1)

        # Debugging: print line position (you can remove or comment this later)
        #print(f"Line Position: {self.line_position}")


    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        # initialize speed and turn
        speed = speed_drive
        turn = 0.0

        if self.line_position is not None:
            # Debugging: print line position (you can remove or comment this later)
            #print(f"Line Position: {self.line_position}, Left Bound: {boundary_left}, Right Bound: {boundary_right}")
            # Determine if the line is on the left, center, or right
            if self.line_position < boundary_left:
                turn = speed_turn  # turn left
            elif self.line_position > boundary_right:
                turn = -speed_turn  # turn right
            else:
                turn = 0.0  # go straight

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):

    print('hello from follow_line (shuttle)')
    rclpy.init(args=args)

    node = LineFollowing()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
