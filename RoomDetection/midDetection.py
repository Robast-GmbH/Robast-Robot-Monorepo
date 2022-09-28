from __future__ import print_function

import os
import cv2 as cv
import numpy as np
import argparse
import random as rng
import yaml

import findDoors


def roommiddle(src):
    rng.seed(1337)
    src = np.uint8(src)
    src = cv.merge((src[:, :, 0], src[:, :, 1], src[:, :, 2]))
    src = src[:, :, 0]
    # Show source image
    cv.namedWindow('Source Image', cv.WINDOW_NORMAL)
    cv.imshow('Source Image', src)
    cv.resizeWindow('Source Image', 2700, 900)
    cv.imwrite("base.png", src)
    #src = cv.imread("base.png")
    #
    cv.waitKey()
    cv.destroyAllWindows()
    #src[np.all(src == 255, axis=2)] = 0
    # np.all()
    # Show output image
    # cv.imshow('Black Background Image', src)
    kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]], dtype=np.float32)

    #_, src = cv.threshold(src, 200, 255, cv.THRESH_BINARY)
    src = cv.adaptiveThreshold(src, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, \
                         cv.THRESH_BINARY, 11, 2)
    #findDoors.locatedoors(src)
    #imgLaplacian = cv.filter2D(src, cv.CV_32F, kernel)
    imgLaplacian = cv.Laplacian(src, 5)
    sharp = np.float32(src)
    imgResult = sharp - imgLaplacian
    # convert back to 8bits gray scale
    imgResult = np.clip(imgResult, 0, 255)
    imgResult = imgResult.astype('uint8')
    imgLaplacian = np.clip(imgLaplacian, 0, 255)
    imgLaplacian = np.uint8(imgLaplacian)
    cv.namedWindow('Laplace Filtered Image', cv.WINDOW_NORMAL)
    cv.imshow('Laplace Filtered Image', imgLaplacian)
    cv.resizeWindow('Laplace Filtered Image', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    cv.namedWindow('New Sharped Image', cv.WINDOW_NORMAL)
    cv.imshow('New Sharped Image', imgResult)
    cv.resizeWindow('New Sharped Image', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()
    #bw = cv.cvtColor(imgResult, cv.COLOR_BGR2GRAY)
    #_, bw = cv.threshold(bw, 40, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    # cv.imshow('Binary Image', bw)
    bw = imgResult
    dist = cv.distanceTransform(bw, cv.DIST_L2, 3)
    # Normalize the distance image for range = {0.0, 1.0}
    # so we can visualize and threshold it
    cv.normalize(dist, dist, 0, 1.0, cv.NORM_MINMAX)

    cv.namedWindow('Distance Transform Image', cv.WINDOW_NORMAL)
    cv.imshow('Distance Transform Image', dist)
    cv.resizeWindow('Distance Transform Image', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    _, dist = cv.threshold(dist, 0.2, 1.0, cv.THRESH_BINARY)
    # Dilate a bit the dist image
    kernel1 = np.ones((3, 3), dtype=np.uint8)
    dist = cv.dilate(dist, kernel1)

    cv.namedWindow('Peaks', cv.WINDOW_NORMAL)
    cv.imshow('Peaks', dist)
    cv.resizeWindow('Peaks', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    dist_8u = dist.astype('uint8')
    # Find total markers
    contours, _ = cv.findContours(dist_8u, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # Create the marker image for the watershed algorithm
    markers = np.zeros(dist.shape, dtype=np.int32)
    # Draw the foreground markers
    for i in range(len(contours)):
        cv.drawContours(markers, contours, i, (i + 1), -1)
    # Draw the background marker
    cv.circle(markers, (5, 5), 3, (255, 255, 255), -1)
    markers_8u = (markers * 10).astype('uint8')

    cv.namedWindow('Markers', cv.WINDOW_NORMAL)

    cv.imshow('Markers', markers_8u)
    cv.resizeWindow('Markers', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    imgResult = cv.merge((imgResult, imgResult, imgResult))
    cv.watershed(imgResult, markers)
    mark = markers.astype('uint8')
    # mark = cv.bitwise_not(mark)

    cv.namedWindow('Markers_v2', cv.WINDOW_NORMAL)
    cv.imshow('Markers_v2', mark)
    cv.resizeWindow('Markers_v2', 2700, 900)
    cv.waitKey()
    cv.destroyAllWindows()

    # findDoors.locatedoors(cv.bitwise_not(imgResult))

    # color rooms

    colors = []
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

    #findDoors.locatedoors(dst)
    mat = recBoundaries(dst)
    mid = calcMiddle(mat)
    printcoordinates(mid)
    return dst


def recBoundaries(img):
    img = np.uint8(img)
    bw = np.zeros(img.shape, dtype=np.int32)
    bw = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    arr = []
    zuordnung = []

    for i in range(bw.shape[0]):
        for j in range(bw.shape[1]):
            index = 0
            if bw[i, j] > 0:
                if any(x == bw[i, j] for x in zuordnung):
                    index = zuordnung.index(bw[i, j])
                else:
                    index = len(zuordnung)
                    zuordnung.append(bw[i, j])
                    arr.append(np.zeros((2, 2, 2)))
                arr[index] = drawboundaries(arr[index], (i, j))
    return arr


def drawboundaries(mat, coordinates):
    if ((mat[0, 0][0] > coordinates[0]) and (mat[0, 0][1] > coordinates[1])) or \
            ((mat[0, 0][0] == 0) and (mat[0, 0][1] == 0)):
        mat[0, 0] = coordinates

    if (mat[1, 0][0] > coordinates[0]) and (mat[1, 0][1] < coordinates[1]):
        mat[1, 0] = coordinates

    if (mat[0, 1][0] < coordinates[0]) and (mat[0, 1][1] < coordinates[1]):
        mat[0, 1] = coordinates

    if (mat[1, 1][0] < coordinates[0]) and (mat[1, 1][1] < coordinates[1]):
        mat[1, 1] = coordinates

    return mat

def calcMiddle(matArray):
    res = []
    for i in matArray:
        tmpx = sum((i[0, 0][0], i[1, 1][0]))/2
        tmpy = sum((i[0, 0][1], i[1, 1][1])) / 2
        res.append((tmpy, tmpx))
    return res

def printcoordinates(coordarray):
    roomNumber = 1
    CenterCoordinatesByRoomNumber = {}
    for i in coordarray:
        # nesting the data like this makes to easy to extend the rooms configuration later,
        # in case we decide to not only store the center point but also the corners, etc.
        data = {
            roomNumber: {
                "center point": {
                    'x': float(i[0]),
                    'y': float(i[1]),
                }
            }
        }
        CenterCoordinatesByRoomNumber.update(data)
        roomNumber += 1

    output = {'rooms': CenterCoordinatesByRoomNumber}
    with open('map_setup.yaml', 'w') as outfile:
        yaml.dump(output, outfile, default_flow_style=False)
