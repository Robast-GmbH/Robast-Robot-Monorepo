import numpy as np
import cv2 as cv

def locatedoors(img):
    mask_left = np.zeros((5, 5), dtype="int")
    mask_left[2, 0] = 1
    mask_left[2, 1] = 1
    mask_left[2, 2] = -1

    mask_right = np.zeros_like(mask_left, dtype="int")
    mask_right[2, 3] = 1
    mask_right[2, 4] = 1
    mask_right[2, 2] = -1

    mask_top = np.zeros_like(mask_left, dtype="int")
    mask_top[0, 2] = 1
    mask_top[1, 2] = 1
    mask_top[2, 2] = -1

    mask_bottom = np.zeros_like(mask_left, dtype="int")
    mask_bottom[3, 2] = 1
    mask_bottom[4, 2] = 1
    mask_bottom[2, 2] = -1

    output_image_left = cv.morphologyEx(img, cv.MORPH_HITMISS, mask_left)
    output_image_right = cv.morphologyEx(img, mask_right)
    output_image_top = cv.morphologyEx(img, cv.MORPH_HITMISS, mask_top)
    output_image_bottom = cv.morphologyEx(img, cv.MORPH_HITMISS, mask_bottom)
    cv.circle(output_image_left, (5, 5), 3, (255, 255, 0), -1)


    cv.imshow(output_image_left)
    cv.waitKey()
    cv.destroyAllWindows()