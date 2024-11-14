import os
import torch
import argparse
import cv2
import numpy as np
from ultralytics import YOLO
from iopaint.helper import load_jit_model, norm_img

def __load_models(detection_models_folder, segmentation_model_path, inpainting_model_path):
    door_segmentation_model = YOLO(segmentation_model_path)
    
    symbol_detection_models = []
    for model_file in os.listdir(detection_models_folder):
        if model_file.endswith('.pt'):
            model_path = os.path.join(detection_models_folder, model_file)
            symbol_detection_models.append(YOLO(model_path))
    
    inpainting_model = load_jit_model(inpainting_model_path, 0, '').eval()
    
    return door_segmentation_model, symbol_detection_models, inpainting_model

def __generate_segmentation_mask_and_inpaint(segmentation_model, image, inpainting_model):
    height, width = image.shape[:2]
    combined_mask = np.zeros((height, width), dtype=np.uint8)
    segmentation_results = segmentation_model.predict(image, task='segment')
    box_offset_margin = 3
    for result in segmentation_results:
        for box in result.boxes.xyxy:
            x1, y1, x2, y2 = map(int, box)
            y1 = max(y1 - box_offset_margin, 0)
            y2 = max(y2 - box_offset_margin, 0)
            cv2.rectangle(combined_mask, (x1, y1), (x2, y2), 255, thickness=cv2.FILLED)

    kernel = np.ones((3, 3), np.uint8)
    combined_mask = cv2.dilate(combined_mask, kernel, iterations=1)
    image = __check_and_resize_image(image, combined_mask)
    inpainted_image = __perform_inpainting(image, combined_mask, inpainting_model)

    return inpainted_image

def __check_and_resize_image(image, mask):
    if image.shape[:2] != mask.shape[:2]:
        image = cv2.resize(image, (mask.shape[0], mask.shape[1]), interpolation=cv2.INTER_LINEAR)
    return image

def __generate_detection_mask_and_inpaint(detection_models, image, inpainting_model, margin=7):
    height, width = image.shape[:2]
    for model in detection_models:
        detection_results = model.predict(image, task='detect')
        for result in detection_results:
            for box in result.boxes.xyxy:
                x1, y1, x2, y2 = map(int, box)
                
                x1 = max(x1 - margin, 0)
                y1 = max(y1 - margin, 0)
                x2 = min(x2 + margin, width - 1)
                y2 = min(y2 + margin, height - 1)
                
                box_mask = np.zeros((height, width), dtype=np.uint8)
                cv2.rectangle(box_mask, (x1, y1), (x2, y2), 255, thickness=cv2.FILLED)
                image = __check_and_resize_image(image, box_mask)
                inpainted_image = __perform_inpainting(image, box_mask, inpainting_model)
                
                image = inpainted_image

    return image

def __perform_inpainting(image, box_mask, inpainting_model):
    
    image_norm = norm_img(image)
    mask_norm = norm_img(box_mask)
    mask_tensor = (mask_norm > 0).astype(np.uint8)
    image_tensor = torch.from_numpy(image_norm).unsqueeze(0).to(0)
    mask_tensor = torch.from_numpy(mask_tensor).unsqueeze(0).to(0)
    inpainted_image = inpainting_model(image_tensor, mask_tensor)
    inpainted_result = inpainted_image[0].permute(1, 2, 0).detach().cpu().numpy()
    inpainted_result = np.clip(inpainted_result * 255, 0, 255).astype("uint8")

    return inpainted_result

def main(args):

    segmentation_model, detection_models, inpainting_model = __load_models(
        os.path.join(os.getcwd(), args.detection_models_folder),
        os.path.join(os.getcwd(), args.segmentation_model_path),
        os.path.join(os.getcwd(), args.inpainting_model_path)
    )
    image = cv2.imread(os.path.join(os.getcwd(), args.input_image_path))
    image = __generate_segmentation_mask_and_inpaint(segmentation_model, image, inpainting_model)
    inpainted_image = __generate_detection_mask_and_inpaint(detection_models, image, inpainting_model)
    cv2.imwrite(os.path.join(os.getcwd(), args.output_image_path), inpainted_image)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inpainting Script with YOLO and Big-LAMA Models")
    parser.add_argument("--detection_models_folder", type=str, default='detection_weights', help="Folder containing detection models")
    parser.add_argument("--segmentation_model_path", type=str, default='door_segment_weight.pt', help="Path to the segmentation model file")
    parser.add_argument("--inpainting_model_path", type=str, default='inpainting_weights.pt', help="Path to the inpainting model file")
    parser.add_argument("--input_image_path", type=str, required=True, help="Path to the input image")
    parser.add_argument("--output_image_path", type=str, default='result.png', help="Path to save the output inpainted image")

    args = parser.parse_args()
    main(args)
