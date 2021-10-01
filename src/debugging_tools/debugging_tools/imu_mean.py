import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import Imu
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile



class SimpleSubscriber(Node):

    def __init__(self):
        # Here we have the class constructor
        # call super() in the constructor in order to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('simple_subscriber')
        # create the subscriber object
        # in this case the subscriptor will be subsribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.subscriber= self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) #is the most used to read LaserScan data and some sensor data too.
        # prevent unused variable warning
        self.subscriber
        # define the variable to save the received info
        self.num_of_imu_data = 0
        self.sum_ax_data = 0
        self.sum_ay_data = 0
        self.sum_az_data = 0
        self.sum_gyr_x_data = 0
        self.sum_gyr_y_data = 0
        self.sum_gyr_z_data = 0
        

    def listener_callback(self, msg):
        # print the log info in the terminal
        self.num_of_imu_data += 1
        self.sum_ax_data += msg.linear_acceleration.x
        self.sum_ay_data += msg.linear_acceleration.y
        self.sum_az_data += msg.linear_acceleration.z
        ax_mean = self.sum_ax_data / self.num_of_imu_data
        ay_mean = self.sum_ay_data / self.num_of_imu_data
        az_mean = self.sum_az_data / self.num_of_imu_data

        self.sum_gyr_x_data += msg.angular_velocity.x
        self.sum_gyr_y_data += msg.angular_velocity.y
        self.sum_gyr_z_data += msg.angular_velocity.z
        gyr_x_mean = self.sum_gyr_x_data / self.num_of_imu_data
        gyr_y_mean = self.sum_gyr_y_data / self.num_of_imu_data
        gyr_z_mean = self.sum_gyr_z_data / self.num_of_imu_data

        msg = 'ax mean: ' + str(format(ax_mean, '.6f')) + ', ay mean:' + str(format(ay_mean, '.6f')) + ', az mean:' + str(format(az_mean, '.6f')) + \
              'gyr_x_mean: ' + str(format(gyr_x_mean, '.6f')) + ', gyr_y_mean:' + str(format(gyr_y_mean, '.6f')) + ', gyr_z_mean:' + str(format(gyr_z_mean, '.6f'))
        self.get_logger().info(msg)
       
        
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_subscriber = SimpleSubscriber()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_subscriber)
    # Explicity destroy the node
    simple_subscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()