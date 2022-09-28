from tkinter import Image

import numpy as np
import cv2 as cv

import midDetection


def roomDetection(img):

    #setup
    #cv.namedWindow('test', cv.WINDOW_NORMAL)
    #cv.imshow('test', img)
    #cv.resizeWindow('test', 1800, 600)
    #cv.waitKey()
    #cv.destroyWindow('test')
    img = cv.morphologyEx(img, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT, (5, 5)))

    midDetection.roommiddle(img)
    return img
