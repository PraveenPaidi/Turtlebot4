import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import numpy
import numpy as np
import scipy.signal as sg
import scipy

MEMORY = 500

my_frame  = numpy.zeros((MEMORY,2))

class IMUDataPlotter(Node):
    labels = ['ax','ay','az']

    def __init__(self):
        super().__init__('imu_data_plotter')
        self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,qos_profile_sensor_data)
        self.subscription  
        self.t0 = None
        self.history = None
        self.fig  = None

    def listener_callback(self, msg):
        self.get_logger().info("received message")

        current_time = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


        if not self.t0:
            self.t0 = current_time

        t = current_time-self.t0

        if self.history is None:
            self.history = numpy.zeros((MEMORY,4))
            self.history1 = numpy.zeros((MEMORY,1))
            self.history2 = numpy.zeros((MEMORY,1))

        self.history[:-1,:] = self.history[1:,:]
        self.history1[:-1,:] = self.history1[1:,:]
        self.history2[:-1,:] = self.history2[1:,:]
        self.history[-1,:] = numpy.array([t, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        
        
        
        if t == 0:
             
            self.x = np.array([[0],[0]])            # state paramaters 
            self.A = np.array([[1, t],[0, 1]])     # state transition matrix 
            self.H = np.array([[1, 0]])             # measurement matrix
            self.HT = np.array([[1],[0]])           # transpose
            self.Q = np.array([[0.05, 0],[0, 0.05]])     # covarinace error matrix
            self.R = 3                               # measurement covarince matrix
            self.P = np.array([[1, 0],[0, 1]])       # covariance matrix   
        
        # Predict State Forward
        self.x_p = numpy.dot(self.A,self.x)
    
        # Predict Covariance Forward
        self.P_p = self.A.dot(self.P).dot(self.A.T) + self.Q
    
        # Correction
        # Compute Kalman Gain
    
        self.S = self.H.dot(self.P_p).dot(self.HT) + self.R
        self.K = self.P_p.dot(self.HT)*(1/self.S)    # gain 
   
        # Estimate State
    
        self.residual = msg.linear_acceleration.x - self.H.dot(self.x_p)    # this is y
        self.x = self.x_p + self.K*self.residual
    
        # Estimate Covariance
        self.P = self.P_p - self.K.dot(self.H).dot(self.P_p)
        
        self.history1[-1,:]=self.x[0] 
        
        self.shape=10
        self.h = sg.get_window('triang', self.shape)
        self.history2[0:MEMORY-self.shape+1,0] = sg.convolve(self.history[:,2], self.h / self.h.sum(),mode='valid')
        self.update(1)
	
    def update(self,i):

        if self.fig is None:
            first = True
            self.fig = plt.figure(figsize=(12,8))
            self.ax = plt.subplot(111)
        else:
            first = False
        self.ax.cla()
        print(self.history[:,0])
        self.ax.plot(self.history[:,0],self.history[:,1],label='Unfiltered data')
        self.ax.plot(self.history[:,0],self.history1[:,0],label='kalman')
        self.ax.plot(self.history[:,0],self.history2[:,0],label='FIR filtered')
        self.ax.set_ylim(self.history[:,1].min(),self.history[:,1].max())    # plot memory
        self.ax.legend()
        self.fig.canvas.draw()
        if first:
            plt.ion()
            plt.show()

def main(args=None):
    rclpy.init(args=args)

    imu_data_plotter = IMUDataPlotter()

    plt.ion()
    plt.show()

    rclpy.spin(imu_data_plotter)

    imu_data_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

