import cv2
import torch
import numpy as np

from ultralytics import YOLO


def segment_image(image):

    # Load the segmentation model
    model = YOLO('')

    inferences = model(image)

    masks = [inference.masks.xy for inference in inferences]
    return masks

def normalise_image(image):

    return cv2.normalize(img=image, dst=None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

def inpaint_image(image,mask):

    # Load inpainting model
    inpaint_model = torch.jit.load('src/map_preprocessing/big-lama.pt')

    # Normalize the image and mask
    image_norm = normalise_image(image)
    mask_norm = normalise_image(mask)

    # Convert normalised images to tensor
    image = torch.from_numpy(image_norm).unsqueeze(0)
    mask =  torch.from_numpy(mask_norm).unsqueeze(0)

    # Feed the mask and image to inapainting model
    inpainted_image = inpaint_model(image,mask)
    
    # Return inpainted image
    cur_res = inpainted_image[0].permute(1, 2, 0).detach().cpu().numpy()
    cur_res = np.clip(cur_res * 255, 0, 255).astype("uint8")
    cur_res = cv2.cvtColor(cur_res, cv2.COLOR_RGB2BGR)
    return cur_res



