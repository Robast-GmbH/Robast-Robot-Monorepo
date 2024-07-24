import torch
import numpy as np
import cv2 as cv

from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from lightglue.superpoint import SuperPoint
from lightglue.lightglue import LightGlue
from lightglue.utils import numpy_image_to_torch, rbd

def ros_msg_to_image(ros_msg):
    image = np.zeros((ros_msg.info.height, ros_msg.info.width), dtype=np.uint8)
    data = ros_msg.data
    index = 0
    for y in range(ros_msg.info.height):
        for x in range(ros_msg.info.width):
            image[y, x] = data[index]
            if data[index] == 0 or data[index] == -1:
                image[y, x] = 255
            else:
                image[y, x] = data[index]
            index += 1
    return image

def calculate_translation_and_rotation(map_msg, feuerplan_image, confidence_threshold, logger):
    # Initialize feature extractor and matcher
    map_image = ros_msg_to_image(map_msg)
    device = "cpu"
    extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
    matcher = LightGlue(features="superpoint").eval().to(device)
    # Extract features from images
    logger.info('Extracting features...')
    feats1 = extractor.extract(numpy_image_to_torch(map_image))
    feats2 = extractor.extract(numpy_image_to_torch(feuerplan_image))
    # Match features between images
    logger.info('Obtaining matches...')
    matches12 = matcher({'image0': feats1, 'image1': feats2})
    # Refine features and matches
    feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
    keypoints1, keypoints2, matches, scores = feats1["keypoints"], feats2["keypoints"], matches12["matches"], matches12['scores']
    #m_keypoints0, m_keypoints1 = keypoints0[matches1[..., 0]], keypoints1[matches1[..., 1]]
    size = (matches12['matches']).size(0)
    if size > 10:
        top_confidences, top_indices = torch.topk(scores, k=8)
        top_matches = matches[top_indices]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        R = calculate_rotation_matrix(points1, points2)
        logger.info(f'{np.linalg.det(R), np.degrees(np.arctan2(R[1, 0], R[0, 0]))}')
        if (top_confidences > confidence_threshold).all():
            world1x, world1y = pixel_coord_to_world_coord(points1.numpy()[0], map_msg.info.resolution, (map_msg.info.origin.position.x, map_msg.info.origin.position.y))
            world2x, world2y = pixel_coord_to_world_coord(points2.numpy()[0], map_msg.info.resolution, (0, 0))
            translation_x = world1x - world2x
            translation_y = world1y - world2y
            logger.info(f'New origin found: ({translation_x}, {translation_y})')
            return translation_x, translation_y, np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        else:
            logger.info('Match confidences below threshold.')
            return None
    else:
        logger.info(f'Not enough matches. Only {size} found.')
        return None

def calculate_rotation_matrix(keypoints1, keypoints2):
    # Calculate the centroids of the keypoints
    centroid1 = np.mean(keypoints1.numpy(), axis=0)
    centroid2 = np.mean(keypoints2.numpy(), axis=0)
    # Normalize the keypoints by subtracting the centroids
    keypoints1_norm = keypoints1.numpy() - centroid1
    keypoints2_norm = keypoints2.numpy() - centroid2
    # Compute the covariance matrix
    H = np.dot(keypoints1_norm.T, keypoints2_norm)
    # Perform SVD
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    # Ensure the rotation matrix is proper (det(R) should be 1)
    if np.linalg.det(R) < 0:
       return None
    else: 
        return R     
   

def pixel_coord_to_world_coord(pixel, resolution, origin):
    world_x = pixel[0] * resolution + origin[0]
    world_y = pixel[1] * resolution + origin[1]
    return world_x, world_y

def publish_occupancy_grid_from_image(publisher, image, translation_and_rotation, logger, clock):
    rotated = ndimage.rotate(image, translation_and_rotation[2], reshape=False)
    cv.imwrite('rotated.jpg', rotated)
    edges = cv.Canny(rotated, 100, 200)
    _, binary_image = cv.threshold(edges, 127, 255, cv.THRESH_BINARY_INV)
    height, width = binary_image.shape
    occupancy_grid = np.zeros((height, width), dtype=np.int8)

    for i in range(height):
        for j in range(width):
            occupancy_grid[i, j] = 0 if binary_image[i, j] == 255 else 100

    data = occupancy_grid.flatten().tolist()
    ros_image_msg = OccupancyGrid()
    ros_image_msg.header.stamp = clock.now().to_msg()
    ros_image_msg.header.frame_id = 'feuerplan'
    ros_image_msg.info.resolution = 0.05
    ros_image_msg.info.width = image.shape[1]
    ros_image_msg.info.height = image.shape[0]
    ros_image_msg.info.origin.position.x = translation_and_rotation[0]
    ros_image_msg.info.origin.position.y = translation_and_rotation[1]
    ros_image_msg.info.origin.orientation.w = 1.0
    ros_image_msg.data = data

    logger.info('Feuerplan published')
    publisher.publish(ros_image_msg)

def create_feuerplan_frame(tf_broadcaster, translation_and_rotation, map_msg, clock):
    transform = TransformStamped()
    transform.header.stamp = clock.now().to_msg()
    transform.header.frame_id = 'feuerplan'
    transform.child_frame_id = 'map'

    transform.transform.translation.x = -translation_and_rotation[0]
    transform.transform.translation.y = -translation_and_rotation[1]
    transform.transform.translation.z = map_msg.info.origin.position.z

    transform.transform.rotation.x = map_msg.info.origin.orientation.x
    transform.transform.rotation.y = map_msg.info.origin.orientation.y
    transform.transform.rotation.z = map_msg.info.origin.orientation.z
    transform.transform.rotation.w = map_msg.info.origin.orientation.w
    tf_broadcaster.sendTransform(transform)