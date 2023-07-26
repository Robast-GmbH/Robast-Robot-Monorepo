import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraRgbInfoNode(Node):

    def __init__(self):
        super().__init__("camera_rgb_info_publisher")
        # Set publisher.
        self.camera_rgb_info_publisher_= self.create_publisher(CameraInfo, "camera_rgb_info",5)
        self.camera_rgb_info_timer = self.create_timer(2.0, self.publish_rgb_camera_info)


    def publish_rgb_camera_info(self):
        # Create default camera info:
        width = 1280.0
        height = 720.0
        camera_rgb_info = CameraInfo()  # This is ROS camera info. Not mine.
        camera_rgb_info.header.stamp = self.get_clock().now().to_msg()
        camera_rgb_info.width = int(width)
        camera_rgb_info.height = int(height)
        camera_rgb_info.header.frame_id = "rb_theron/base_footprint/front_top_oak_d_rgb_camera_rgb_optical_frame"
        camera_rgb_info.distortion_model = "plumb_bob"
        camera_rgb_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_rgb_info.d = [0.080921, -0.244505, 0.006909, 0.015070, 0.000000]
        camera_rgb_info.k = [1007.03765, 0.0, 693.05655,0.0, 1007.59267, 356.9163,0.0, 0.0, 1.0]
        camera_rgb_info.p = [984.22742, 0.0, 720.52955, 0.0,
                        0.0, 1022.02051, 359.96275, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        self.camera_info_publisher_.publish(camera_rgb_info)

class CameraRightInfoNode(Node):

    def __init__(self):
        super().__init__("camera_right_info_publisher")
        self.camera_right_info_publisher_= self.create_publisher(CameraInfo, "camera_right_info",5)
        self.camera_right_info_timer = self.create_timer(2.0, self.publish_right_camera_info)


    def publish_right_camera_info(self):
        # Create default camera info:
        width = 640.0
        height = 480.0
        camera_right_info = CameraInfo()  # This is ROS camera info. Not mine.
        camera_right_info.header.stamp = self.get_clock().now().to_msg()
        camera_right_info.width = int(width)
        camera_right_info.height = int(height)
        camera_right_info.header.frame_id = "rb_theron/base_footprint/front_top_oak_d_right_camera_optical_frame"
        camera_right_info.distortion_model = "plumb_bob"
        camera_right_info.r = [ 0.99912007, -0.00732194,  0.04129749,
          0.00754073,  0.99995833, -0.00514455,
         -0.0412581 ,  0.00545144,  0.99913365]
        camera_right_info.d = [0.027691, -0.084910, -0.002828, 0.001984, 0.000000]
        camera_right_info.k = [801.10787,   0.     , 661.27048,
           0.     , 802.27862, 362.70452,
           0.     ,   0.     ,   1.     ]
        camera_right_info.p = [ 833.04045,    0.     ,  616.04111, -242.47655,
            0.     ,  833.04045,  359.7206 ,    0.     ,
            0.     ,    0.     ,    1.     ,    0.     ]
        self.camera_right_info_publisher_.publish(camera_right_info)


def main(args=None):
    rclpy.init(args=args)
    rgb_camera_info_node = CameraRgbInfoNode()
    right_camera_info_node = CameraRightInfoNode()
    rclpy.spin(right_camera_info_node)
    rclpy.spin(rgb_camera_info_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()