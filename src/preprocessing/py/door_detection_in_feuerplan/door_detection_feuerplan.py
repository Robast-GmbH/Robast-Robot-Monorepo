import numpy as np

from dataclasses import dataclass
from ultralytics import YOLO

@dataclass
class DoorDetectionResult:
    door_frame_points: list[list[tuple[float,float]]]
    robot_nav_points: list[list[tuple[float,float]]]

class FeuerplanDoorDetector:
    def __init__(self, image_path:str, model_path:str, distance_from_door_frame:int, confidence_threshold:float):
        self.image_path = image_path
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.distance_from_door_frame = distance_from_door_frame
        self.door_segmentation_model = YOLO(self.model_path)

    def segment_doors_in_feuerplan(self) -> list:
        """
        Segments doors in a feuerplan.

            Output:
                segmentation_result.masks.xy: A list containing points on the image that defines doors.
        """

        segmentation_results = self.door_segmentation_model.predict(source=self.image_path, conf=self.confidence_threshold)
        for segmentation_result in segmentation_results:
            return segmentation_result.masks.xy
    
    def obtain_points_from_segmentation_results(self) -> DoorDetectionResult:
        """
        Calculates the door frame and robot navigation points on the feuerplan.

            Output:
                DoorDetectionResult: An instance containing points that define end of door frame and navigation points for each identified door.

        """

        door_points = self.segment_doors_in_feuerplan()
        door_frame_points = []
        robot_nav_points = []
        for door_point in door_points:
            door_frame_points.append([tuple(door_point[0]), tuple(door_point[-1])])
            midpoint = (door_point[0] + door_point[-1]) / 2
            direction_vector = door_point[-1] - door_point[0]
            perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])
            perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector) * self.distance_from_door_frame
            point_in_front = midpoint + perpendicular_vector
            point_behind = midpoint - perpendicular_vector
            robot_nav_points.append([tuple(point_in_front), tuple(point_behind)])

        return DoorDetectionResult(
           door_frame_points=door_frame_points,
           robot_nav_points=robot_nav_points        
       )