import numpy as np


def read_pgm(pgmf):
    header = pgmf.readline()
    assert header[:2] == b'P5'
    skip = pgmf.readline()
    header = pgmf.readline()
    (width, height) = [int(i) for i in header.split()[0:2]]
    header = pgmf.readline()
    depth = int(header)
    assert depth <= 255
    img = np.zeros([height, width]);

    for y in range(height):
        for x in range(width):
            img[y, x] = ord(pgmf.read(1))
    return img
