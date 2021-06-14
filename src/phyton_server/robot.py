

# System modules
from datetime import datetime

# 3rd party modules
from flask import make_response, abort

#not API
def get_timestamp():
    return datetime.now().strftime(("%Y-%m-%d %H:%M:%S"))

def robotSelection( ):
    return aktiveRobots[0]

# Data to serve with our API
aktiveRobots = []
aktiveRobots.append("Robi")
aktiveRobots.append("Basti")
 
def robots():
    return []

def navToRoom(waypoints,lrobot):
    for wp in waypoints:
        print(wp)

def randomNavToRoom(waypoints):
    navToRoom(waypoints,robotSelection())

def addWaypoint(waypoints, lrobot):
    print("Waypoints are added to "+lrobot+".")

    
def addTask(task, lrobot, waypoints= None):
    print("1 Task is added.\n")

def randomAddTask(task, waypoints= None):
    addTask(task,robotSelection(),waypoints)

def getTask(lrobot):
    return "not implemented"

def getPrio(lrobot):
    return -1

def getTimeToIdle(lrobot):
    return -1

def getIdleBots():
    return []



    
        
