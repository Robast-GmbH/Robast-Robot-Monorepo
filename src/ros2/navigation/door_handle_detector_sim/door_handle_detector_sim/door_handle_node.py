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

        self.bridge = CvBridge()

        # Load model
        weights = 'src/navigation/door_handle_detector_sim/weights/door_4.pt'  # model.pt path(s)
        self.model = yolov5.models.common.DetectMultiBackend(weights, dnn=False, fp16=False)

        self.rgb_camera_image_subscription_ = self.create_subscription(
            Image,
            '/back_top_realsense_camera/color',
            self.rgb_camera_callback,
            10)
        self.depth_camera_image_subscription_ = self.create_subscription(
            Image, '/back_top_realsense_camera/depth', self.callback_depth_img, 10)
        self.door_handle_detection_publisher_ = self.create_publisher(
            SpatialDetectionArray, 'stereo/door_handle_position', 10)

        self.depth_camera_image_subscription_
        self.rgb_camera_image_subscription_

    def depth_img_callback(self, data):
        self.cv_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

    def rgb_camera_callback(self, data):

        # Convert ROS message to BGR image
        img_from_msg = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Convert to rgb image and normalize before feeding into the detection network
        preprocessed_image = self.preprocess_image(img_from_msg)

        # Inference
        prediction = self.model(preprocessed_image)

        # Apply Non Max Suppression to prediction
        prediction = yolov5.utils.general.non_max_suppression(prediction, conf_thres=0.4, iou_thres=0.45)

        spatial_detection_array = SpatialDetectionArray()

        spatial_detection_array.header.stamp = self.get_clock().now().to_msg()
        spatial_detection_array.header.frame_id = 'rb_theron/base_footprint/back_top_realsense_camera_color_link'

        # Camera characteristics
        fx = 1007.03765
        fy = 1007.59267
        cx = 693.05655
        cy = 356.9163

        for i, detection in enumerate(prediction):
            # Extract bounding box coordinates, class_id and confidence of each detection
            for *xyxy, confidence, class_id in detection:
                
                # Convert bounding box coordinates to xywh
                xywh = yolov5.utils.general.xyxy2xywh(torch.tensor(xyxy).view(1, 4)).view(-1).tolist()
                
                # Coordinates of center of bounding box in depth image
                dist = float(self.cv_depth_image[int(xywh[1]), int(xywh[0])])

                # Fill ´SpatialDetection´ msg    
                spatial_detection = SpatialDetection()
                hypo = ObjectHypothesis()
                hypo.class_id = str(class_id)
                hypo.score = float(confidence)
                spatial_detection.results.append(hypo)
                spatial_detection.bbox.size_x = xywh[2]
                spatial_detection.bbox.size_y = xywh[3]
                spatial_detection.position.x = dist * (xywh[0] - cx) / fx
                spatial_detection.position.y = dist * (xywh[1] - cy) / fy
                spatial_detection.position.z = dist
                spatial_detection.is_tracking = False
                spatial_detection_array.detections.append(spatial_detection)

        self.door_handle_detection_publisher_.publish(spatial_detection_array)

    def preprocess_image(self, image):

        image = image[np.newaxis, :, :, :]
        image = image[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB
        image = np.ascontiguousarray(image)

        image = torch.from_numpy(image)
        image = image.float()
        normalized_image = image / 255

        return normalized_image


def main(args=None):
    rclpy.init(args=None)
    door_handle_detector = Door_Handle_Detection()
    rclpy.spin(door_handle_detector)
    door_handle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()