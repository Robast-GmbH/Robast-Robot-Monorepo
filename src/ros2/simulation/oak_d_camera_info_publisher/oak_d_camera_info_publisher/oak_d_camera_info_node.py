import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraRgbInfoNode(Node):

    def __init__(self):
        super().__init__("camera_rgb_info_publisher")
       
       # Calibration parameters
        self.image_height = 1280
        self.image_width = 720
        self.distortion_model = "plumb_bob"
        self.camera_matrix = [1007.03765, 0.0, 693.05655,0.0, 1007.59267, 356.9163,0.0, 0.0, 1.0] 
        self.projection_matrix = [984.22742, 0.0, 720.52955, 0.0, 0.0, 1022.02051, 359.96275, 0.0,0.0, 0.0, 1.0, 0.0]
        self. distortion_parameters = [0.080921, -0.244505, 0.006909, 0.015070, 0.000000]
        self.rectification_matrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.header_stamp = self.get_clock().now().to_msg()

        # Create publishers
        self.back_camera_rgb_info_publisher_= self.create_publisher(CameraInfo, 'back_top_oak_d_camera/color_camera_info',10)
        self.front_camera_rgb_info_publisher_= self.create_publisher(CameraInfo, "front_top_oak_d_camera/color_camera_info",10)
        self.camera_rgb_info_timer = self.create_timer(0.09, self.publish_rgb_camera_info)

    def publish_rgb_camera_info(self):
        
        # Set hard coded values for calibration information of OAK-D-S2 camera
        
        back_camera_rgb_info =  front_camera_rgb_info = CameraInfo()  # Create a sensor message for back oak-d-camera
        front_camera_rgb_info.header.stamp = back_camera_rgb_info.header.stamp = self.header_stamp
        front_camera_rgb_info.width = back_camera_rgb_info.width = self.image_height
        front_camera_rgb_info.height = back_camera_rgb_info.height = self.image_height
        back_camera_rgb_info.header.frame_id = "rb_theron/base_footprint/back_top_oak_d_camera_rgb_camera_optical_frame"
        front_camera_rgb_info.header.frame_id = "rb_theron/base_footprint/front_top_oak_d_camera_rgb_camera_optical_frame"
        front_camera_rgb_info.distortion_model = back_camera_rgb_info.distortion_model = self.distortion_model
        front_camera_rgb_info.r = back_camera_rgb_info.r = self.rectification_matrix
        front_camera_rgb_info.d = back_camera_rgb_info.d = self.distortion_parameters
        front_camera_rgb_info.k = back_camera_rgb_info.k = self.camera_matrix
        front_camera_rgb_info.p = back_camera_rgb_info.p = self.projection_matrix
        self. back_camera_rgb_info_publisher_.publish(back_camera_rgb_info)
        self. front_camera_rgb_info_publisher_.publish(front_camera_rgb_info)
        


def main(args=None):
    rclpy.init(args=args)
    rgb_camera_info_node = CameraRgbInfoNode()
    rclpy.spin(rgb_camera_info_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()