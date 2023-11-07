import sys
from PIL import Image
import os

TILE_SIZE = 25

def png_to_pixel_images(input_image_path, output_folder):
    # Open the input PNG image
    img = Image.open(input_image_path)

    # Get the width and height of the image
    width, height = img.size

    # Calculate the number of tiles in the x and y directions
    num_tiles_x = width // TILE_SIZE
    num_tiles_y = height // TILE_SIZE

    # Loop through each tile
    for tile_x in range(num_tiles_x):
        for tile_y in range(num_tiles_y):
            # Calculate the coordinates of the tile in the original image
            left = tile_x * TILE_SIZE
            upper = tile_y * TILE_SIZE
            right = left + TILE_SIZE
            lower = upper + TILE_SIZE

            # Crop the tile from the original image
            tile_img = img.crop((left, upper, right, lower))

            # Create the folder structure (folder/x/y) for the tile image
       

            # Save the tile image in the corresponding folder with a unique name
            tile_img_path = os.path.join(output_folder, f"tile_{tile_x}_{tile_y}.png")
            tile_img.save(tile_img_path)

if __name__ == "__main__":
    # Check if the correct number of arguments is provided
    if len(sys.argv) != 3:
        print("Usage: python png_to_pixel_images.py input_image_path output_folder")
    else:
        input_image_path = sys.argv[1]  # Get the input image path from the first argument
        output_folder = sys.argv[2]     # Get the output folder from the second argument

        png_to_pixel_images(input_image_path, output_folder)
