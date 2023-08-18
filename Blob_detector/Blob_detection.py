import sys
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError




class BlobDetector(Node):

    def __init__(self, thr_min, thr_max, blur=7, detection_window=None):
        super().__init__('Blob_detector')
    
        self._threshold = [thr_min, thr_max]
        self._blur = blur
        self.detection_window = detection_window
        self._t0 = time.time()
        self.blob_point = Point()
   
        print (">> Publishing position to topic /blob/point")
        self.publisher_ = self.create_publisher(Point, '/blob/pub', 10)
        self.br = CvBridge()
        self.subscription = self.create_subscription(Image,'/color/image', self.listener_callback, 10)  
        print ("<< Subscribed to topic /color/image")
        
    def listener_callback(self,data):
        current_frame = self.br.imgmsg_to_cv2(data)
         
        ## Convert to hsv color space ##
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        ## Obtain the mask ##
        red_mask = cv2.inRange(hsv_frame, self._threshold[0],self._threshold[1])
        
        #cv2.imshow("Red blob Mask",red_mask)
        min_size = 100  # 1000
                
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        red_contours = [c for c in red_contours if cv2.contourArea(c) > min_size]
        
        red_blobs_1 = [(cv2.contourArea(c), cv2.moments(c)) for c in red_contours]
        #red_blobs_1.sort(key=lambda x: x, reverse=True)
        
        print('lol')
        for cont in red_blobs_1:
            self.blob_point.x = cont[1]['m10']/cont[1]['m00']
            self.blob_point.y = cont[1]['m01']/cont[1]['m00']
            
            print("X : ",self.blob_point.x)
            #print("y : ",self.blob_point.y)
            print("Area :",cont[0])
        
            if cont[0] >= 800:
                self.publisher_.publish(self.blob_point)
        
        #cv2.imshow("image",current_frame)
        cv2.waitKey(1)
        
def main(args=None):

    rclpy.init(args=args)
    red_lower = np.array([160, 100, 20])  #0
    red_upper = np.array([179, 255, 255]) #10
    image_subscriber = BlobDetector(red_lower,red_upper)
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
