import torch
import numpy as np
import cv2 as cv

from LightGlue.lightglue import LightGlue, SuperPoint, DISK, SIFT, ALIKED, DoGHardNet, viz2d
from LightGlue.lightglue.utils import load_image, rbd

image1 = load_image("/workspace/src/map_preprocessing/output.jpg")
image2 = load_image("/workspace/src/map_preprocessing/tiplu_6OG.jpg")

#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = "cpu"

extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(features="superpoint").eval().to(device)

feats1 = extractor.extract(image1.to(device))
feats2 = extractor.extract(image2.to(device))

matches12 = matcher({"image0": feats1, "image1": feats2})
#print(matches12)
feats0, feats1, matches01 = [
    rbd(x) for x in [feats1, feats2, matches12]
]

kpts1, kpts2, matches = feats1["keypoints"], feats2["keypoints"], matches12["matches"]

kpc1, kpc2 = viz2d.cm_prune(matches12["prune0"]), viz2d.cm_prune(matches12["prune1"])
points0 = feats0['keypoints'][matches[..., 0]]  
points1 = feats1['keypoints'][matches[..., 1]]  
print(points0, points1)


