import argparse
import cv2

from clean_feuerplan.cleaning_mechnaism import segment_image, inpaint_image

def main():

    parser = argparse.ArgumentParser()

    # Add command-line arguments
    parser.add_argument('-i', '--feuerplan_image', dest='input_file', required=True, help='Feuerplan image file path')
    parser.add_argument('-o', '--output', dest='output_file', default='cleaned_feuerplan.png', help='Output path to cleaned feuerplan')
    parser.add_argument('-s', '--segmentation_only', action='store_true', help='Shows segmentation results')

    # Parse command-line arguments
    args = parser.parse_args()

    # Access the parsed arguments
    input_file = args.input_file
    output_file = args.output_file
    segmentation_only = args.segmentation_only

    # Check if the user specified to show only segmentation results
    if segmentation_only:
        # Perform image segmentation
        segmentation_masks = segment_image(input_file)
        
        # Save segmentation results as an image
        cv2.imwrite(str('merged_segs.jpg'), segmentation_masks.cpu().numpy())
    else:
        # Perform and save inpainting on the feuerplan image
        processed_feuerplan = inpaint_image(input_file)
        cv2.imwrite(str('cleaned_feuerplan.jpg'), processed_feuerplan)

    print('Feuerplan pre-processing completed successfully.')

if __name__ == '__main__':