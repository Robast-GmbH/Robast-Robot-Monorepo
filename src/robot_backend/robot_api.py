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
    return {"success":ros_bridge.nav_bridge.navigate_to_goal_pose("robot", (x, y, z))}


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal():
    return {"success":ros_bridge.nav_bridge.cancel_navigate_to_goal_pose()}


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigating": ros_bridge.nav_bridge.is_navigating()}


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time():
    return {"remaining_seconds": ros_bridge.nav_bridge.get_remaining_nav_time()}

@app.get("/modules", tags=["Modules"])
def read_modules():
    module_data = {}
    for module in ros_bridge.module_bridge.modules:
        is_open = ros_bridge.module_bridge.context.get(f'drawer_is_open_{module.module_id}_{module.drawer_id}')
        module_data[f'{module.module_id}_{module.drawer_id}'] = {"is_open": is_open, "is_e_drawer": module.is_e_drawer}

    return module_data

@app.post("/open_drawer", tags=["Modules"])
def post_open_drawer(module_id: int, drawer_id: int):
    return {"success":ros_bridge.module_bridge.open_drawer(module_id, drawer_id)}

@app.post("/close_drawer", tags=["Modules"])
def post_close_drawer(module_id: int, drawer_id: int):
    return {"success":ros_bridge.module_bridge.close_drawer(module_id, drawer_id)}


uvicorn.run(app,host="192.168.0.200", port=8001)
