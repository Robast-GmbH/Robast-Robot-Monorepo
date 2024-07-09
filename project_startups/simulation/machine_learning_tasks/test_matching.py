import torch
import cv2 as cv
import numpy as np
from lightglue import viz2d
from lightglue.lightglue import LightGlue
from lightglue.utils import numpy_image_to_torch, rbd
from lightglue.superpoint import SuperPoint
from lightglue.sift import SIFT

def preprocess_image(feuerplan_image:np.ndarray) -> np.ndarray:
        processed_image = cv.GaussianBlur(feuerplan_image, (3, 3), 3, 3)
        processed_image = cv.Canny(processed_image, 50, 150, 3)
        dilation_size = 1
        dilation_element = cv.getStructuringElement(cv.MORPH_RECT, (2 * dilation_size + 1, 2 * dilation_size + 1))
        processed_image = cv.dilate(processed_image, dilation_element)
        feuerplan_image &= processed_image
        _, preprocessed_feuerplan = cv.threshold(feuerplan_image, 150, 127, cv.THRESH_BINARY)
        return preprocessed_feuerplan

def get_three_best_matches(confidence_values:torch.tensor, matches:torch.tensor, keypoints1:torch.tensor, keypoints2:torch.tensor) -> tuple[float,float]:
        top_values, top_indices = torch.topk(confidence_values, k=3)
        top_matches = matches[top_indices]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2, matches, top_values

image1 = cv.imread("/workspace/src/machine_learning_tasks/feuerplan_publisher/images/slide2.png",cv.IMREAD_GRAYSCALE)
normalized_image = np.clip(image1.astype(np.int16) - 128, -128, 127)
cv.imwrite("n.png",normalized_image)
image2 = cv.imread("/workspace/output_image_opencv.png") 
#image1 = preprocess_image(image1)
device = "cpu" 
extractor = SuperPoint(max_num_keypoints = None).eval().to(device)
matcher = LightGlue(features = "superpoint").eval().to(device)
# Extract features from images

feats1 = extractor.extract(numpy_image_to_torch(normalized_image))
feats2 = extractor.extract(numpy_image_to_torch(image2))
# Match features between images

matches12 = matcher({'image0': feats1, 'image1': feats2})
# Refine features and matches
feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
print(matches12['scores'])
points1, points2, matches, top_values = get_three_best_matches(matches12['scores'],matches12['matches'], feats1["keypoints"], feats2["keypoints"])
m_kpts1, m_kpts2 = feats1['keypoints'][matches12['matches'][..., 0]], feats2['keypoints'][matches12['matches'][..., 1]] 
axes = viz2d.plot_images([normalized_image, image2])
viz2d.plot_matches(points1, points2, color="lime", lw=0.2)
viz2d.save_plot("out.png")