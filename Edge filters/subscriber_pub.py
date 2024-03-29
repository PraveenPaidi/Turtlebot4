import rclpy
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
#backSub = cv2.createBackgroundSubtractorMOG2('/home/praveenpaidi/ros2_ws/bag_files/first.png') 

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('subscriber')
    self.backSub = cv2.createBackgroundSubtractorMOG2(history=10**9,varThreshold=200) 
    self.subscription = self.create_subscription(Image,'video_frames',self.listener_callback,10)
    self.publisher_ = self.create_publisher(Image, 'vid_frames', 10)
    self.br = CvBridge()
  
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera", current_frame)  
    fgMask = self.backSub.apply(current_frame) 
    self.publisher_.publish(self.br.cv2_to_imgmsg(fgMask))
    #cv2.imshow("fgMask", fgMask)  
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
