import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraRgbInfoNode(Node):

    def __init__(self):
        super().__init__("camera_rgb_info_publisher")
       
       # Calibration parameters. Interface of CameraInfo message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        self.camera_config = CameraInfo()
        self.camera_config.width = 1280
        self.camera_config.height = 720
        self.camera_config.distortion_model = "plumb_bob"
        self.camera_config.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]                                        # Rectification matrix
        self.camera_config.d = [0.080921, -0.244505, 0.006909, 0.015070, 0.000000]                                  # Distortion parameters
        self.camera_config.k = [1007.03765, 0.0, 693.05655,0.0, 1007.59267, 356.9163,0.0, 0.0, 1.0]                 # Camera matrix
        self.camera_config.p = [984.22742, 0.0, 720.52955, 0.0, 0.0, 1022.02051, 359.96275, 0.0,0.0, 0.0, 1.0, 0.0] # Projection matrix
                

        # Create publishers
        self.back_camera_rgb_info_publisher_= self.create_publisher(CameraInfo, 'back_top_oak_d_camera/color_camera_info',10)
        self.front_camera_rgb_info_publisher_= self.create_publisher(CameraInfo, "front_top_oak_d_camera/color_camera_info",10)
        self.camera_rgb_info_timer = self.create_timer(0.09, self.publish_rgb_camera_info)

    def publish_rgb_camera_info(self):
        # Update the time stamp for the camera info
        self.camera_config.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the rgb camera info for the back oak-d-camera
        back_camera_rgb_info = self.camera_config
        back_camera_rgb_info.header.frame_id = "rb_theron/base_footprint/back_top_oak_d_camera_rgb_camera_optical_frame"
        self.back_camera_rgb_info_publisher_.publish(back_camera_rgb_info)
        
        # Publish the rgb camera info for the front oak-d-camera
        front_camera_rgb_info = self.camera_config
        front_camera_rgb_info.header.frame_id = "rb_theron/base_footprint/back_top_oak_d_camera_rgb_camera_optical_frame"
        self.front_camera_rgb_info_publisher_.publish(front_camera_rgb_info)
        


def main(args=None):
    rclpy.init(args=args)
    rgb_camera_info_node = CameraRgbInfoNode()
    rclpy.spin(rgb_camera_info_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()