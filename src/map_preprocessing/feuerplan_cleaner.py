import os
import cv2
import torch

from lightglue import LightGlue, SuperPoint, DISK
from lightglue.utils import load_image, rbd
from lightglue import viz2d

device = torh.device("cuda" if torch.cuda.is_available() else "cpu")
extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(features="superpoint").eval().to(device)

features_of_first_image = extractor.extract(first_image.to(device))
features_of_second_image = extractor.extract(second_image.to(device))
matches_1 = matcher