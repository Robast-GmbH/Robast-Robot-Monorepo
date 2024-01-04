import cv2
import torch
import numpy as np

from ultralytics import YOLO


def segment_image(image):

    # Load the segmentation model
    segmentation_model = YOLO('src/map_preprocessing/clean_feuerplan/segment_weight.pt')

    # Segment image
    segementation_result = segmentation_model.predict(image)

    #print(segementation_result[0].masks.data)

    mask = segementation_result[0].masks.data

    mask = torch.any(mask, dim=0).int() * 255

    # Return masks of the segmented image
    return mask

def inpaint_image(image_path):
    # Read the input image
    image = cv2.imread(image_path)
    
    # Segment the image to create a mask
    mask = segment_image(image)
    
    # Load the inpainting model
    inpaint_model = torch.jit.load('src/map_preprocessing/clean_feuerplan/big-lama.pt')

    # Normalize the input image and mask
    image = norm_img(image)
    mask = norm_img(mask.cpu().numpy())
    
    # Convert the mask to binary (0 or 1)
    mask = (mask > 0) * 1

    # Convert the normalized image and mask to PyTorch tensors and move them to the GPU
    image = torch.from_numpy(image).unsqueeze(0).to(torch.cuda.current_device())
    mask = torch.from_numpy(mask).unsqueeze(0).to(torch.cuda.current_device())

    # Inpaint the image using the loaded model
    inpainted_image = inpaint_model(image, mask)
    inpainted_image = inpainted_image[0].permute(1, 2, 0).detach().cpu().numpy()
    inpainted_image = np.clip(inpainted_image * 255, 0, 255).astype("uint8")
    inpainted_image = cv2.cvtColor(inpainted_image, cv2.COLOR_RGB2BGR)

    return inpainted_image

def norm_img(np_img):
    # Check if the image is grayscale (2D)
    if len(np_img.shape) == 2:
        # If grayscale, add a third dimension to represent channels
        np_img = np_img[:, :, np.newaxis]

    # Transpose the image to have channels as the first dimension
    np_img = np.transpose(np_img, (2, 0, 1))

    # Convert image data type to float32 and normalize pixel values to the range [0, 1]
    np_img = np_img.astype("float32") / 255

    # Return the normalized image
    return np_img