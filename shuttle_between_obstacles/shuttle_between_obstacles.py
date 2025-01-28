import rclpy
import rclpy.node
import cv2
import numpy as np
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ShuttleLogic(rclpy.node.Node):

    def __init__(self):
        super().__init__('shuttle_logic')
        self.declare_parameter('FollowLineSpeed', 1.0)
        self.declare_parameter('FollowLineTurn', 0.5)
        self.declare_parameter('FollowObjectSpeed', 1.0)
        self.declare_parameter('FollowObjectTurn', 0.5)
        self.FollowLineSpeed = 0.0
        self.FollowLineTurn = 0.0
        self.FollowObjectTurn = 0.0
        self.FollowObjectSpeed = 0.0

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            Twist,
            'InputStateMachineFollowLine',
            self.InputFollowLineCallback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        self.subscription = self.create_subscription(
            Twist,
            'InputStateMachineFollowObject',
            self.InputFollowObjectCallback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.TimerCallback)

    def InputFollowLineCallback(self, msg):
        print("entering input follow line callback...") 
        self.FollowLineSpeed = msg.linear.x
        self.FollowLineTurn = msg.angular.z
        speed = msg.linear.x
        turn = msg.angular.z
        #print(f"line input callback speed: {speed}, turn: {turn}")
    def InputFollowObjectCallback(self,msg):
        print("entering input follow object callback...") 
        self.FollowObjectSpeed = msg.linear.x
        self.FollowObjectTurn = msg.angular.z
        speed = msg.linear.x
        turn = msg.angular.z
        print(f"object intput callback speed: {speed}, turn: {turn}")
   
    def TimerCallback(self):
        print("entering timercallback")
        follow_line_speed = self.FollowLineSpeed
        follow_line_turn = self.FollowLineTurn
        follow_object_speed = self.FollowObjectSpeed
        follow_object_turn = self.FollowObjectTurn
        msg = Twist()
        if(follow_object_speed != 0.0):
            speed = float(follow_line_speed)
            print(f"supposed to follow the line(drive) at speed {speed}")
            turn = float(follow_line_turn)
            msg.linear.x = float(speed)
            msg.angular.z = float(turn)
            self.publisher_.publish(msg)
        elif(follow_object_speed == 0.0): #follow_object_speed = 0, meaning robot is facing a close obstacle
            speed = follow_object_speed
            print(f"supposed to stop, speed: {speed}") 
            turn = 0.5
            # create message
            msg.linear.x = float(speed)
            msg.angular.z = float(turn)
            self.publisher_.publish(msg) 
            turn_duration = 3.1415926535 * 2
            print(f"turnduration: {turn_duration}")
            time.sleep(turn_duration) 
            print("done turning")
            self.FollowObjectSpeed = 0.3
            self.FolowObjectTurn = 0.0
        else:
            print(f"undefined behavior!!!")
            return
                
def main(args = None):
    rclpy.init(args=args)

    node = ShuttleLogic()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
