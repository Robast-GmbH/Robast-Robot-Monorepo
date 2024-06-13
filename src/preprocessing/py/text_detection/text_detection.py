import cv2
import numpy as np

from dataclasses import dataclass
from paddleocr import PaddleOCR

@dataclass
class TextDetectionResult:
    bounding_boxes: list[list[tuple[float, float]]]
    bounding_box_centers: list[tuple[float, float]]
    text_detections: list[str]
    scores: list[float]

class OCRTextDetector:
    def __init__(self, image_path:str, language = 'german'):
        self.image_path = image_path
        self.language = language
        self.ocr_model = PaddleOCR(use_angle_cls=True, lang=self.language, show_log=False)

    def find_center_of_bounding_box(self,bounding_box_corners:list[list[float, float]]) -> tuple[float, float]:
        """
        Calculates the center coordinates of a bounding box.

            Input:
            bounding_box_corners: A list of corner coordinates of the bounding box, where each corner is represented as [x, y].

            Output:
                (center_x, center_y): Center coordinates of the bounding box.

        """
        x_coords = [point[0] for point in bounding_box_corners]
        y_coords = [point[1] for point in bounding_box_corners]
        
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        
        return center_x, center_y

    def detect_text(self) -> TextDetectionResult:
        """
        Detects text in an image using an OCR model.

            Input:
                image_path: Path to input image.

            Output:
                TextDetectionResult: An instance containing bounding boxes, detected texts, and confidence scores.
        """
        detection_result = self.ocr_model.ocr(self.image_path, cls=True)
        detection_result = detection_result[0]
        boxes = [line[0] for line in detection_result]
        centers = [self.find_center_of_bounding_box(box) for box in boxes]
        text_detections = [line[1][0] for line in detection_result]
        confidence_scores = [line[1][1] for line in detection_result]


        return TextDetectionResult(
            bounding_boxes=boxes,
            bounding_box_centers=centers,
            text_detections=text_detections,
            scores=confidence_scores
        )
    
    def visualise_bounding_box(self, output_path:str, box:list[list[float, float]], color=(0, 0, 255), thickness=2) -> None:
        """
        Visualizes a bounding box on an image. Stores the image with bounding box on it at `output_path`

            Input:
                image_path: Path to input image.
                output_path: Path where the image is to be stored after drawing the bounding box
                box: A list of corner coordinates of the bounding box, where each corner is represented as [x, y].
                color (optional): The color of the bounding box. Defaults to red (0, 0, 255).
                thickness (optional): The thickness of the bounding box lines. Defaults to 2.
        """
        image = cv2.imread(self.image_path)
        pts = np.array(box, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)
        cv2.imwrite(output_path,image)