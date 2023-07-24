import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
from pathlib import Path
import yolov5
import torch
import torch.backends.cudnn as cudnn

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.weights='src/navigation/door_handle/resource/door_4.onnx'  # model.pt path(s)
        self.imgsz=(640, 480)  # inference size (pixels)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.line_thickness=3  # bounding box thickness (pixels)
        self.hide_labels=False  # hide labels
        self.hide_conf=False  # hide confidences
        self.half=False  # use FP16 half-precision inference
        self.stride = 32
        device_num=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.dnn = False
        self.data= 'data/coco128.yaml'  # dataset.yaml path
        self.half=False  # use FP16 half-precision inference
        self.augment=False  # augmented inferenc

        # Initialize
        #self.device = select_device(device_num)

        # Load model
        self.model = yolov5.models.common.DetectMultiBackend(self.weights, dnn=self.dnn, data=self.data, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = yolov5.utils.general.check_img_size(self.imgsz, s=stride)

        # Run inference
        bs = 1  # batch_size
        self.model.warmup(imgsz=(1 if pt or self.model.triton else bs, 3, *imgsz))  # warmup

        self.subscription = self.create_subscription(
            Image,
            '/front_realsense_camera/color',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        # Inference
        visualize = yolov5.utils.general.increment_path(save_dir / Path(path).stem, mkdir=True) if self.visualize else False
        pred = self.model(img, augment=self.augment, visualize=visualize)

        # Apply NMS
        pred = yolov5.utils.general.non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string

            annotator = yolov5.utils.plots.Annotator(img0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                det[:, :4] = yolov5.utils.general.scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=yolov5.utils.plots.colors(c, True))

        cv2.imshow("IMAGE", img0)
        cv2.waitKey(4)


def main(args=None):
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()