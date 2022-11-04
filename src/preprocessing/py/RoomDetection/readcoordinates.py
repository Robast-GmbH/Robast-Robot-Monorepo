import numpy as np

def readCoordinates(filename):
    testsite_array = []
    with open(filename) as my_file:
        for line in my_file:
            testsite_array.append(line)
    res = []
    for i in testsite_array:
        i = i.replace('\n', '')
        i = i.replace('[ ', '[')
        i = i.replace('[ ', '[')
        i = i.replace(']', '')
        i = i.replace('[', '')
        i = i.replace('  ', ' ')
        tmpx, tmpy = i.split(' ')
        res.append((int(tmpx), int(tmpy)))

    return res