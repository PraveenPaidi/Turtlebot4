import rclpy
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('subscriber')
    self.subscription = self.create_subscription(Image,'/color/preview/image', self.listener_callback, 10)
    self.br = CvBridge()
    self.i=0
    self.kernel_3x3 = numpy.ones ((7,7),numpy.float32) / (7*7)
    self.kernel_sharp = numpy.array([[-1 ,-1, -1],[-1,9,-1],[-1,-1,-1]])
    self.kernel_Gaussian = numpy.array([[1/16 ,1/8, 1/16],[1/8,1/4,1/8],[1/16,1/8,1/16]])


  def listener_callback(self, data):

    self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera", current_frame)
    
    blurred = cv2.filter2D(current_frame , -1 , self.kernel_3x3)
    cv2.imshow("blur", blurred)
    sharpened= cv2.filter2D(current_frame , -1 , self.kernel_sharp)
    Gaussian_blur= cv2.filter2D(current_frame , -1 , self.kernel_Gaussian)
    Canny_filter = cv2.Canny(current_frame, 50 , 120)
    cv2.imshow("sharp" , sharpened)
    cv2.imshow("Gaussian_blur" , Gaussian_blur)
    cv2.imshow("Canny_filter" , Canny_filter)
    
    if self.i%50==1:
      cv2.imwrite('Original'+str(self.i)+'.jpg',current_frame)
      cv2.imwrite('Blurred'+str(self.i)+'.jpg',blurred)
      cv2.imwrite('Sharpened'+str(self.i)+'.jpg',sharpened)
      cv2.imwrite('Gaussian_blur'+str(self.i)+'.jpg',Gaussian_blur)
      cv2.imwrite('Canny_filter'+str(self.i)+'.jpg',Canny_filter)
    self.i+=1      
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
