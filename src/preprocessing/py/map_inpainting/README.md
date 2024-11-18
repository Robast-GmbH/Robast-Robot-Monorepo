Map Preprocessing runs segmentation to generate segmentation masks and remove the objects within the masks

To run the segmentation + inpainting:
 cd map_inapinting
 python3 inpainting.py --input_image_path <path to the feuerplan image>

Outputs cleaned feuerplan image as `result.png`