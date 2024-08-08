import os
import argparse

from door_detection import FeuerplanDoorDetector

def main():
    
    parser = argparse.ArgumentParser(description="Door Detector Script")  
    parser.add_argument('-i','--image_path', type=str, required=True, help='path to input image')  
    parser.add_argument('-m','--model_path', type=str, required=True, help='path to model')  
    parser.add_argument('-o','--output_path', type=str, default='result.png', help='path to store results')
    parser.add_argument('-d','--distance_from_door_frame', type=int, default=50, help='Abstand vom Türrahmen in Pixeln')  
    parser.add_argument('-c','--confidence_threshold', type=float, default=0.5, help='Confidence Threshold für Türerkennung')  

    args = parser.parse_args()  

    image_path = os.path.join(os.getcwd(), args.image_path)
    model_path = os.path.join(os.getcwd(), args.model_path)
    output_path = os.path.join(os.getcwd(), args.output_path)

    # Initialize the door detector with the provided arguments.  
    door_detector = FeuerplanDoorDetector(  
        image_path=image_path,  
        model_path=model_path,  
        distance_from_door_frame=args.distance_from_door_frame,  
        confidence_threshold=args.confidence_threshold  
    )  

    # Perform door detection and get results  
    detection_result = door_detector.obtain_points_from_detection_results()  
    print(detection_result)  

    # Save the results to a file
    door_detector.visualize_and_save(detection_result=detection_result, output_path=output_path)

if __name__ == "__main__":  
    main()  