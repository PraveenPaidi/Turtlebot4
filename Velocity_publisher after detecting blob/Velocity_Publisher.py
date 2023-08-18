import cv2
import numpy as np
import rclpy
import math, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
#from team_messages.msg import Blob
from geometry_msgs.msg      import Point


class Velocity_publisher(Node):
    def __init__(self):
        super().__init__('Velocity_publisher')
        print(">> Publishing velocity commands")
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel = Twist()
        self.images = Image()
        
        print(">> Subscribing to the blobs")
        self.x = 0.0
        self.y = 0.0
        self.frame_w = 0.0
        self.frame_h = 0.0
        self.image_sub = self.create_subscription(Point,'/blob/pub', self.lol, 10)
        self.im_sub = self.create_subscription(Image,'/color/image',self.rizz,10)
        self.timer_ = self.create_timer(0.1, self.wheel_movement)
        self.br = CvBridge()
        
    def lol(self,msg):
        self.x = msg.x
        self.y = msg.y
    def rizz(self,msg):
        self.images = self.br.imgmsg_to_cv2(msg)
        self.frame_w = msg.width / 2
        self.frame_h = msg.height / 2
        
    def wheel_movement(self):     
        print("X : ",self.x)
        print(self.x)    
        print('lololo')
        self.steering =  2.0 * self.x
        print(self.steering)
        print('done')
        if self.steering <= 0:
            self.steering = 0.0
        elif self.steering >= 1.5:
            self.steering = 0.1
        
        center_offset = 0
        center_offset = self.x - self.frame_w
        print('center offset',center_offset)
        if center_offset >= 50:
            print('centre value is greater than 100')
            self.throttle = 0.0005 * center_offset
        elif center_offset <= -50:
            print('center value id lesse than 100')
            self.throttle = -0.0005 * center_offset
        else:
            self.throttle = 0.0   
        print("Throttle values", self.throttle)
        self.vel.linear.x = self.steering
        print(self.steering)
        self.vel.angular.z = self.throttle  
        print(self.throttle)
        self.vel_pub.publish(self.vel)
        print(self.vel)
            
            
def main(args=None):
    rclpy.init(args=args)
    node = Velocity_publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
