from fastapi import FastAPI
from ros_bridge import RosBridge
import uvicorn

ros_bridge = RosBridge(ip="localhost", port=9090)
app = FastAPI()


@app.get("/")
def read_root():
    return "API is running"


@app.get("/is_ros_bridge_connected")
def read_is_ros_bridge_connected():
    return {"is_connected": ros_bridge.ros.is_connected}


@app.get("/robot_pos", tags=["Robot Position"])
def read_robot_pos():
    return ros_bridge.robot_pos_bridge.get_robot_pos()


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(x: float, y: float, z: float):
    return ros_bridge.nav_bridge.navigate_to_goal_pose("robot", (x, y, z))


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal():
    return ros_bridge.nav_bridge.cancel_navigate_to_goal_pose()


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigating": ros_bridge.nav_bridge.is_navigating()}


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time():
    return {"remaining_seconds": ros_bridge.nav_bridge.get_remaining_nav_time()}


uvicorn.run(app, port=8001)
