import torch
import numpy as np
import cv2 as cv

from LightGlue.lightglue import LightGlue, SuperPoint, DISK, SIFT, ALIKED, DoGHardNet, viz2d
from LightGlue.lightglue.utils import load_image, rbd

image1 = load_image("src/map_preprocessing/LightGlue/assets/DSC_0410.JPG")
image2 = load_image("src/map_preprocessing/LightGlue/assets/DSC_0411.JPG")

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

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
viz2d.plot_images([image1, image2])
viz2d.plot_keypoints([kpts1, kpts2])
#kpts1_np, kpts2_np = kpts1.cpu().numpy(), kpts2.cpu().numpy()
#print(kpts1)
#m_kpts1, m_kpts2 = kpts1[matches[..., 0]], kpts2[matches[..., 1]]


# src_img = cv.imread("src/map_preprocessing/tiplu_6OG.pgm")
# dst_img = cv.imread("src/map_preprocessing/ceaned.jpg")
# transformation_matrix, _ = cv.estimateAffinePartial2D(kpts1_np, kpts2_np)
# transformed_image = cv.warpAffine(dst_img, transformation_matrix, (dst_img.shape[1], dst_img.shape[0]))


