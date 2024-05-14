from fastapi import FastAPI
from ros_bridge import RosBridge

ros_bridge = RosBridge(ip="localhost", port=9090)
app = FastAPI()


"""
======================
General API Endpoints
======================
"""


@app.get("/")
def read_root():
    return "API is running"


"""
===========================
Robot Status API Endpoints
===========================
"""


@app.get("/is_ros_bridge_connected", tags=["Robot Status"])
def read_is_ros_bridge_connected():
    return {"is_connected": ros_bridge.ros.is_connected}


@app.get("/robot_pos", tags=["Robot Status"])
def read_robot_pos():
    return ros_bridge.robot_pos_bridge.get_robot_pos()


"""
=========================
Navigation API Endpoints
=========================
"""


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(x: float, y: float, z: float):
    return {"success": ros_bridge.nav_bridge.navigate_to_goal_pose("robot", (x, y, z))}


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal():
    return {"success": ros_bridge.nav_bridge.cancel_navigate_to_goal_pose()}


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigating": ros_bridge.nav_bridge.is_navigating()}


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time():
    return {"remaining_seconds": ros_bridge.nav_bridge.get_remaining_nav_time()}


"""
======================
Modules API Endpoints
======================
"""


@app.get("/modules", tags=["Modules"])
def read_modules():
    return ros_bridge.module_bridge.get_modules()


@app.post("/open_drawer", tags=["Modules"])
def post_open_drawer(module_id: int, drawer_id: int):
    return {"success": ros_bridge.module_bridge.open_drawer(module_id, drawer_id)}


@app.post("/close_drawer", tags=["Modules"])
def post_close_drawer(module_id: int, drawer_id: int):
    return {"success": ros_bridge.module_bridge.close_drawer(module_id, drawer_id)}
