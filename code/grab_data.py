import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy
import yaml


class IMUDataLogger(Node):
    labels = ['ax','ay','az']

    def __init__(self):
        super().__init__('imu_data_logger')
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

        print(t)

        if self.history is None:
            self.history = numpy.array([[t, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]])
        else:
            self.history = numpy.concatenate([self.history,numpy.array([[t, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]])],0)


def main(args=None):
    rclpy.init(args=args)

    imu_data_logger = IMUDataLogger()

    try:
        rclpy.spin(imu_data_logger)
#    except SystemExit:
#        print('exiting')
    except KeyboardInterrupt:
        mydata = imu_data_logger.history.tolist()
        mydict = {'data':mydata,}
        with open('file.yaml','w') as f:
            yaml.dump(mydict,f)

    imu_data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

