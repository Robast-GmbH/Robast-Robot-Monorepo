from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from ros_bridge import RosBridge
from models.module_process_data import ModuleProcessData

door_available = False
ros_bridge = RosBridge(ip="localhost", port=9090, door_available=door_available)
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


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
    return {"success": ros_bridge.nav_bridge.navigate_to_goal_pose((x, y, z))}


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal():
    return {"success": ros_bridge.nav_bridge.cancel_navigate_to_goal_pose()}


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigating": ros_bridge.nav_bridge.is_navigating()}


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time():
    return {"remaining_seconds": ros_bridge.nav_bridge.get_remaining_nav_time()}


@app.post("/block_navigation", tags=["Navigation"])
def post_block_nav():
    return {"success": ros_bridge.nav_bridge.block_nav()}


@app.post("/unblock_navigation", tags=["Navigation"])
def post_unblock_nav():
    return {"success": ros_bridge.nav_bridge.unblock_nav()}


@app.get("/is_navigation_blocked", tags=["Navigation"])
def read_is_nav_blocked():
    return {"is_nav_blocked": ros_bridge.nav_bridge.is_nav_blocked}


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


@app.post("/start_module_process", tags=["Modules"])
def post_start_module_process(
    data: ModuleProcessData,
):
    return {
        "success": ros_bridge.module_bridge.start_module_process(
            data.module_id, data.drawer_id, data.process_name
        )
    }


@app.post("/finish_module_process", tags=["Modules"])
def post_finish_module_process():
    return ros_bridge.module_bridge.finish_module_process()


@app.get("/module_process_status", tags=["Modules"])
def get_module_process_status():
    return {"success": ros_bridge.module_bridge.get_current_module_process()}


"""
======================
Doors API Endpoints
======================
"""

if door_available:

    @app.post("/open_door", tags=["Doors"])
    def post_open_drawer():
        return {"success": ros_bridge.door_bridge.open_door()}

    @app.post("/close_door", tags=["Doors"])
    def post_close_drawer():
        return {"success": ros_bridge.door_bridge.close_door()}
