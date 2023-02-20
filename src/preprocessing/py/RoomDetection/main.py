# This is a sample Python script.

# Press Umschalt+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np

import readPgm
import readMapSetup
import rommDetection
import roomDetection2
import cv2 as cv


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    f = open("C:/Robast/foxy_ws/src/map_server/maps/5OG.pgm", 'rb')
    im = readPgm.read_pgm(f)
    f.close()
    im = cv.imread("cleaned_grundriss.png")
    cv.namedWindow('test', cv.WINDOW_NORMAL)
    cv.imshow('test', im)
    cv.resizeWindow('test', 2700, 900)
    cv.waitKey()
    cv.destroyWindow('test')
    a = rommDetection.roomDetection(im)
