import cv2
import numpy as np
import matplotlib.pyplot as plt

from typing import List, Tuple
from dataclasses import dataclass
from ultralytics import YOLO

@dataclass
class DoorDetectionResult:
    door_frame_points: list[list[tuple[float,float]]]
    robot_nav_points: list[list[tuple[float,float]]]

class FeuerplanDoorDetector:
    def __init__(self, image_path:str, model_path:str, distance_from_door_frame:int, confidence_threshold:float) -> None:
        self.__image_path = image_path
        self.__model_path = model_path
        self.__confidence_threshold = confidence_threshold
        self.__distance_from_door_frame = distance_from_door_frame
        # Load the YOLO model with the specified path and configuration.
        self.__door_detection_model = YOLO(self.__model_path)

    def __find_obb_doors_in_feuerplan(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Detects oriented bounding boxes around doors in a feuerplan.
            Output:
                box_points: A list containing points on the image that defines bounding boxes around doors.
                box_angles: A list containing angles (in degrees) of the bounding boxes.
        """
        detection_results = self.__door_detection_model(source=self.__image_path, conf=self.__confidence_threshold)
        box_points = detection_results[0].obb.xyxyxyxy.cpu().numpy()
        box_angles = detection_results[0].obb.xywhr.cpu().numpy()[:, 4] * (180 / np.pi)
        return box_points, box_angles
    
    def obtain_points_from_detection_results(self) -> DoorDetectionResult:
        """
        Calculates the door frame and robot navigation points on the feuerplan.
            Output:
                DoorDetectionResult: An instance containing points that define end of door frame and navigation points for each identified door.

        """
        detection_boxes, detection_box_angles = self.__find_obb_doors_in_feuerplan()
        door_frame_points = self.__find_all_door_frame_points(boxes=detection_boxes, angles=detection_box_angles)
        robot_nav_points = [self.__perpendicular_projections(point1, point2, self.__distance_from_door_frame) for point1, point2 in door_frame_points]
        return DoorDetectionResult(
           door_frame_points=door_frame_points,
           robot_nav_points=robot_nav_points        
       )  
    
    def __adjust_rotation_and_find_lower_points(self, points:np.ndarray, angle:float, center:np.ndarray) -> List[Tuple[float, float]]:
        """
        Sorts the bounding box points.
            Input:      
                points: A numpy array containing points of the bounding box.
                angle: A float value representing the angle of the bounding box.
                center: A numpy array containing the center of the bounding box.
            Output:
                sorted_points: A list containing points of the bounding box bottom up.

        """
        if 150 <= angle <= 210:
            angle = 180
        elif 80 <= angle <= 100:
            angle = 90
        r_inv = self.__inverse_rotation_matrix(angle)
        points_centered = points - center
        unrotated_points = r_inv @ points_centered.T
        unrotated_points = unrotated_points.T + center
        if angle == 90:
            sorted_points = sorted(unrotated_points, key=lambda y: y[0], reverse=True)[:2]
        elif angle == 180:
            sorted_points = sorted(unrotated_points, key=lambda x: x[1], reverse=False)[:2]
        else:
            sorted_points = sorted(unrotated_points, key=lambda x: x[1], reverse=True)[:2]
        return [tuple(point) for point in sorted_points]
    
    def __inverse_rotation_matrix(self, angle:float) -> np.ndarray:
        """
        Calculates the inverse of the rotation matrix.
            Input:      
                angle: A float value representing the angle of the bounding box.
            Output:
                np.array: A 2x2 numpy array containing the inverse of the rotation matrix.
        """
        angle_rad = np.radians(-angle) 
        cos = np.cos(angle_rad)
        sin = np.sin(angle_rad)
        return np.array([[cos, -sin], [sin, cos]])
    
    def __find_all_door_frame_points(self, boxes:np.ndarray, angles:np.ndarray) -> List[List[Tuple[float, float]]]:
        """
        Processes multiple bounding boxes and angles to find the door frame points.
            Input:
                boxes: A numpy array containing points of the bounding boxes.
                angles: A numpy array containing angles of the bounding boxes. 
            Output:     
                all_door_points: A list containing points of the door frame for each identified door.
        """
        all_door_points = []
        for box, angle in zip(boxes, angles):
            points = np.array(box)
            center = np.mean(points, axis=0)
            door_points = self.__adjust_rotation_and_find_lower_points(points, angle, center)
            all_door_points.append(door_points)
        return all_door_points
    
    def __perpendicular_projections(self, point1:Tuple[float,float], point2:Tuple[float,float], distance:int) -> Tuple[Tuple[float,float], Tuple[float,float]]:
        """
        Calculates the perpendicular projections of the door frame points to find the robot navigation points.
            Input:
                point1: A tuple containing the first point of the door frame.
                point2: A tuple containing the second point of the door frame.
                distance: An integer representing the distance from the door frame.
            Output:
                point_in_front, point_behind: A tuple containing the front and behind points of the robot navigation.
        """
        point1 = np.array(point1)
        point2 = np.array(point2)
        midpoint = (point1 + point2) / 2
        direction_vector = point2 - point1
        perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
        perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector) * distance
        point_in_front = midpoint + perpendicular_vector
        point_behind = midpoint - perpendicular_vector
        return tuple(point_in_front), tuple(point_behind)
    
    def visualize_and_save(self, detection_result: DoorDetectionResult, output_path: str) -> None:
        """
        Visualizes the door frame and robot navigation points on the feuerplan and saves the image.
                Input:
                    output_path: A string representing the path to save the image.
        """
        door_frame_points = detection_result.door_frame_points
        robot_nav_points = detection_result.robot_nav_points
        image = cv2.imread(self.__image_path)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(10, 8))
        plt.imshow(image_rgb)
        for door_points, nav_points in zip(door_frame_points, robot_nav_points):
            # Plot door frame points
            door_x, door_y = zip(*door_points)
            plt.scatter(door_x, door_y, color = 'blue')
            # Plot navigation points
            front, behind = nav_points
            nav_x, nav_y = zip(front, behind)
            plt.scatter(nav_x, nav_y, color='green')
        plt.title("Door Frame and Robot Navigation Points")
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(output_path)
        plt.close()