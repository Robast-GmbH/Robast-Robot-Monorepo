This project contains code for door detection and finding nav points in a feuerplan.

Example usage:
    Make sure to install required libraries.

    def main():
        # Initialize the door detector with the path to the image and model.
        image_path = 'src/preprocessing/py/door_detection_in_feuerplan/FRP_Musterplan.svg.png'
        model_path = '/home/sagar/Monorepo/src/preprocessing/py/door_detection_in_feuerplan/door_detector_weights.pt'
        distance_from_door_frame = 50  # distance in pixels to calculate navigation points
        confidence_threshold = 0.5  # confidence threshold for door detection

        door_detector = FeuerplanDoorDetector(
            image_path=image_path,
            model_path=model_path,
            distance_from_door_frame=distance_from_door_frame,
            confidence_threshold=confidence_threshold
        )

        # Perform door detection and get results
        detection_result = door_detector.obtain_points_from_detection_results()
