from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ros_bridge import RosBridge

ros_bridge = RosBridge(ip="localhost", port=9090)
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


@app.get("/robot_lost", tags=["Robot Status"])
def read_robot_lost():
    return ros_bridge.robot_pos_bridge.get_robot_lost()


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


@app.get("/is_navigation_completed", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigation_completed": ros_bridge.nav_bridge.is_navigation_completed()}


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
    return {"is_nav_blocked": ros_bridge.nav_bridge.get_is_nav_blocked()}


@app.get("/requires_replan", tags=["Navigation"])
def read_requires_replan():
    return {"requires_replan": ros_bridge.nav_bridge.requires_replan()}


@app.post("/set_initial_point", tags=["Navigation"])
def post_set_initial_point(x: float, y: float, z: float):
    return {"success": ros_bridge.robot_pos_bridge.set_initial_point(x, y, z)}


"""
======================
Modules API Endpoints
======================
"""


@app.get("/submodule_status", tags=["Modules"])
def read_submodule_status(module_id: int, submodule_id: int):
    return {
        "is_open": ros_bridge.module_bridge.get_submodule_is_open(
            module_id, submodule_id
        ),
        "is_stall_guard_triggered": ros_bridge.module_bridge.get_submodule_stall_guard_triggered(
            module_id, submodule_id
        ),
        "opening_timed_out": ros_bridge.error_bridge.received_drawer_not_opened_error(),
    }


@app.post("/open_submodule", tags=["Modules"])
def post_open_submodule(module_id: int, submodule_id: int):
    return {"success": ros_bridge.module_bridge.open_submodule(module_id, submodule_id)}


@app.post("/close_submodule", tags=["Modules"])
def post_close_submodule(module_id: int, submodule_id: int):
    return {
        "success": ros_bridge.module_bridge.close_submodule(module_id, submodule_id)
    }


"""
======================
NFC API Endpoints
======================
"""


@app.get("/nfc_tag", tags=["NFC"])
def get_nfc_tag():
    return {"nfc_tag": ros_bridge.nfc_bridge.get_nfc_tag()}


@app.get("/read_nfc_tag", tags=["NFC"])
def read_nfc_tag(timeout_in_s: int):
    return ros_bridge.nfc_bridge.read_nfc_tag(timeout_in_s=timeout_in_s)


"""
======================
Disinfection API Endpoints
======================
"""


@app.get("/disinfection_triggered", tags=["Disinfection"])
def get_disinfection_triggered(timeout: int):
    return ros_bridge.disinfection_module_bridge.wait_for_disinfection_triggered(
        timeout
    )
