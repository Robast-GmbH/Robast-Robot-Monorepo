import yaml
from PIL import Image
import schemas

[x,y] = Image.open("/workspace/src/base.png").size

def readMapSetup(filename):
    with open(filename, 'r') as file:
        return yaml.load(file, Loader=yaml.FullLoader)

def convertclientCoords(goal: schemas.Goal):
    goal.x = goal.x * (x / goal.clientX )
    goal.y = goal.y * (y / goal.clientY)
    goal.clientX = x
    goal.clientY = y

    return goal
