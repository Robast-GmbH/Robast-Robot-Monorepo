import numpy as np
import yaml

def readMapSetup(filename):
    with open(filename, 'r') as file:
        map_setup = yaml.load(file, Loader=yaml.FullLoader)
        print(map_setup)


if __name__ == '__main__':
    readMapSetup('map_setup.yaml')