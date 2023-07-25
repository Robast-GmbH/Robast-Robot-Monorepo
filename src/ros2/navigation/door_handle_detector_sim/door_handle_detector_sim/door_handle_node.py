import rclpy
import yolov5
import torch

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesis
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from cv_bridge import CvBridge


class Door_Handle_Detection(Node):

    def __init__(self):
        super().__init__('door_handle_detection')

        weights='src/navigation/door_handle_detector_sim/weights/door_4.pt'  # model.pt path(s)
        self.bridge = CvBridge()
        self.cv_depth_image = np.zeros((640, 480), np.uint8)

        # Load model
        self.model = yolov5.models.common.DetectMultiBackend(weights, dnn=False, fp16=False)

        self.rgb_camera_image_subscription_ = self.create_subscription(
            Image,
            '/back_top_realsense_camera/color',
            self.rgb_camera_callback,
            10)
        self.depth_camera_image_subscription_ = self.create_subscription(Image,'/back_top_realsense_camera/depth', self.callback_depth_img,10)
        self.door_handle_detection_publisher_= self.create_publisher(SpatialDetectionArray, 'stereo/door_handle_position',10)

        self.depth_camera_image_subscription_  
        self.rgb_camera_image_subscription_

    def callback_depth_img(self, data):
        self.cv_depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def rgb_camera_callback(self, data):

        # Convert ROS message to BGR image
        rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Camera characterstics 
        fx = 1007.03765
        fy = 1007.59267
        cx = 693.05655
        cy = 356.9163

        rgb_img = rgb_img[np.newaxis, :, :, :]        
        rgb_img = rgb_img[..., ::-1].transpose((0, 3, 1, 2))            # BGR to RGB
        rgb_img = np.ascontiguousarray(rgb_img)

        rgb_img = torch.from_numpy(rgb_img).to(self.model.device)
        rgb_img = rgb_img.float()
        rgb_img /= 255                            

        # Inference
        pred = self.model(rgb_img)

        # Apply NMS
        pred = yolov5.utils.general.non_max_suppression(pred, conf_thres = 0.4, iou_thres = 0.45)

        detection_array = SpatialDetectionArray()
        detection = SpatialDetection()
        hypo = ObjectHypothesis()

        for i, det in enumerate(pred): 
            # Extract bounding box coordinates, class_id and confidence of each detection
            for *xyxy, confidence, class_id in det:
                hypo.class_id = str(class_id)
                hypo.score = float(confidence)  
                detection.results.append(hypo)
                # Convert bounding box coordinates to xywh
                xywh = yolov5.utils.general.xyxy2xywh(torch.tensor(xyxy).view(1, 4)).view(-1).tolist() 
                detection.bbox.size_x = xywh[2]
                detection.bbox.size_y = xywh[3]
                # Coordinates of center of bounding box in depth image   
                dist = float(self.cv_depth_image[int(xywh[1]), int(xywh[0])])
                detection.position.x = dist*(xywh[0] - cx)/fx
                detection.position.y = dist*(xywh[1] - cy)/fy
                detection.position.z = dist
                detection.is_tracking = False
                detection_array.header.stamp = self.get_clock().now().to_msg()
                detection_array.header.frame_id = 'rb_theron/base_footprint/back_top_realsense_camera_color_link'
                detection_array.detections.append(detection)
        
        self.door_handle_detection_publisher_.publish(detection_array)


def main(args=None):
    rclpy.init(args=None)
    door_handle_detector = Door_Handle_Detection()
    rclpy.spin(door_handle_detector)
    door_handle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()