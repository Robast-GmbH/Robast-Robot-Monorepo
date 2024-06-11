from paddleocr import PaddleOCR
import cv2
import numpy as np

def visualise_bounding_box(image_path:str, output_path:str, box:list, color=(0, 0, 255), thickness=2):
    """
    Visualizes a bounding box on an image. Stores the image with bounding box on it at `output_path`

        Input:
            image_path: Path to input image.
            output_path: Path where the image is to be stored after drawing the bounding box
            box: A list of corner coordinates of the bounding box, where each corner is represented as [x, y].
            color (optional): The color of the bounding box. Defaults to red (0, 0, 255).
            thickness (optional): The thickness of the bounding box lines. Defaults to 2.
    """
    image = cv2.imread(image_path)
    pts = np.array(box, np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)
    cv2.imwrite(output_path,image)

def find_center_of_bounding_box(bounding_box_corners:list):
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

def detect_text(image_path:str, ocr_model):
    """
    Detects text in an image using an OCR model.

        Input:
            image_path: Path to input image.
            ocr_model: The OCR model instance used for text detection.

        Output(dict):
            bounding_boxes: List of bounding box coordinates for detected texts.
            text_detections: List of detected text strings.
            scores: List of confidence scores for detected text.
    """
    detection_result = ocr_model.ocr(image_path, cls=True)
    detection_result = detection_result[0]
    boxes = [line[0] for line in detection_result]
    text_detections = [line[1][0] for line in detection_result]
    confidence_scores = [line[1][1] for line in detection_result]

    return {
            "bounding boxes": boxes,
            "text detections": text_detections,
            "scores": confidence_scores
        }

def search_text_in_image(image_path:str, search_term:str, language:str):
    """
    Searches for a specific text in an image
    
    Input:
        image_path: Path to input image.
        search_term: Text to search for in the image.
        language: Language to use for text detection.

    Output:
        (center_x, center_y): Center coordinates of the bounding box around the `search_term`.    
    """
    ocr = PaddleOCR(use_angle_cls=True, lang=language, show_log=False) # Downloads the model once
    detection_result = detect_text(image_path=image_path,ocr_model=ocr)
    index_of_interest = detection_result["text detections"].index(search_term)
    # Uncomment this line to visualise the bounding box around the text
    #visualise_bounding_box(image_path=image_path,box=detection_result["bounding boxes"][index_of_interest])
    print(detection_result["bounding boxes"][index_of_interest].type())
    return find_center_of_bounding_box(detection_result["bounding boxes"][index_of_interest])