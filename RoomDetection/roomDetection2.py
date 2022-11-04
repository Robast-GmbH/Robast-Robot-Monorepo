import cv2 as cv
import numpy as np
import argparse
import random as rng

def roomDetection2(img):
    rng.seed(1337)
    src = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    cv.namedWindow('Source Image', cv.WINDOW_NORMAL)
    cv.imshow('Source Image', src)
    cv.resizeWindow('Source Image', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    src = cv.adaptiveThreshold(src, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, \
                               cv.THRESH_BINARY, 11, 2)

    # noise removal
    kernel = np.ones((3, 3), np.uint8)
    opening = cv.morphologyEx(src, cv.MORPH_OPEN, kernel, iterations=2)
    # sure background area
    sure_bg = cv.dilate(opening, kernel, iterations=3)

    cv.namedWindow('background', cv.WINDOW_NORMAL)
    cv.imshow('background', sure_bg)
    cv.resizeWindow('background', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    # Finding sure foreground area
    dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
    ret, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
    # Finding unknown region
    sure_fg = np.uint8(sure_fg)

    cv.namedWindow('forground', cv.WINDOW_NORMAL)
    cv.imshow('forground', sure_fg)
    cv.resizeWindow('forground', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    unknown = cv.subtract(sure_bg, sure_fg)

    # Marker labelling
    ret, markers = cv.connectedComponents(sure_fg)
    # Add one to all labels so that sure background is not 0, but 1
    markers = markers + 1
    # Now, mark the region of unknown with zero
    markers[unknown == 255] = 0

    markers = cv.watershed(img, markers)
    img[markers == -1] = [255, 0, 0]

    colors = []
    contours, _ = cv.findContours(sure_fg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        colors.append((rng.randint(0, 256), rng.randint(0, 256), rng.randint(0, 256)))
    dst = np.zeros((markers.shape[0], markers.shape[1], 3), dtype=np.uint8)
    for i in range(markers.shape[0]):
        for j in range(markers.shape[1]):
            index = markers[i, j]
            if index > 0 and index <= len(contours):
                dst[i, j, :] = colors[index - 1]
    cv.namedWindow('Final Result', cv.WINDOW_NORMAL)
    cv.imshow('Final Result', dst)
    cv.resizeWindow('Final Result', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    cv.namedWindow('Result', cv.WINDOW_NORMAL)
    cv.imshow('Result', img)
    cv.resizeWindow('Result', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()