from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ros_bridge import RosBridge
from models.logger import Logger
from log_router import log_router

api_logger = Logger("api", "api.log")
ros_bridge = RosBridge(ip="localhost", port=9090)
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(log_router, prefix="/logs")

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


@app.get("/battery_status", tags=["Robot Status"])
def read_battery_status():
    return ros_bridge.robot_status_bridge.get_battery_status()


@app.get("/emergency_stop_pressed", tags=["Robot Status"])
def read_emergency_stop_pressed():
    return ros_bridge.robot_status_bridge.get_emergency_stop_pressed()


"""
=========================
Navigation API Endpoints
=========================
"""


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(x: float, y: float, z: float, use_reorientation: bool = False):
    api_logger.info(
        f"Received goal pose: {x}, {y}, {z}, use_reorientation: {use_reorientation}"
    )
    return {
        "success": ros_bridge.nav_bridge.navigate_to_goal_pose(
            (x, y, z), use_reorientation
        )
    }


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal():
    api_logger.info("Received cancel goal request")
    return {"success": ros_bridge.nav_bridge.cancel_navigate_to_goal_pose()}


@app.get("/is_navigation_completed", tags=["Navigation"])
def read_is_navigating():
    return {"is_navigation_completed": ros_bridge.nav_bridge.is_navigation_completed()}


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time():
    return {"remaining_seconds": ros_bridge.nav_bridge.get_remaining_nav_time()}


@app.post("/block_navigation", tags=["Navigation"])
def post_block_nav():
    api_logger.info("Received block navigation request")
    return {"success": ros_bridge.nav_bridge.block_nav()}


@app.post("/unblock_navigation", tags=["Navigation"])
def post_unblock_nav():
    api_logger.info("Received unblock navigation request")
    return {"success": ros_bridge.nav_bridge.unblock_nav()}


@app.get("/is_navigation_blocked", tags=["Navigation"])
def read_is_nav_blocked():
    return {"is_nav_blocked": ros_bridge.nav_bridge.get_is_nav_blocked()}


@app.get("/requires_replan", tags=["Navigation"])
def read_requires_replan():
    return {"requires_replan": ros_bridge.nav_bridge.requires_replan()}


@app.post("/set_initial_point", tags=["Navigation"])
def post_set_initial_point(x: float, y: float, z: float):
    api_logger.info(f"Received set initial point request: {x}, {y}, {z}")
    return {"success": ros_bridge.robot_pos_bridge.set_initial_point(x, y, z)}


"""
======================
Modules API Endpoints
======================
"""


@app.get("/living_devices", tags=["Modules"])
def read_living_devices():
    return ros_bridge.module_bridge.get_living_devices()


@app.get("/submodule_status", tags=["Modules"])
def read_submodule_status(module_id: int, submodule_id: int):
    return ros_bridge.module_bridge.get_submodule_state(module_id, submodule_id)


@app.post("/open_submodule", tags=["Modules"])
def post_open_submodule(module_id: int, submodule_id: int):
    api_logger.info(f"Received open submodule request: {module_id}, {submodule_id}")
    return {"success": ros_bridge.module_bridge.open_submodule(module_id, submodule_id)}


@app.post("/close_submodule", tags=["Modules"])
def post_close_submodule(module_id: int, submodule_id: int):
    api_logger.info(f"Received close submodule request: {module_id}, {submodule_id}")
    return {
        "success": ros_bridge.module_bridge.close_submodule(module_id, submodule_id)
    }


"""
======================
NFC API Endpoints
======================
"""


@app.get("/read_nfc_tag", tags=["NFC"])
def read_nfc_tag(timeout_in_s: int):
    api_logger.info(f"Received read NFC tag request with timeout: {timeout_in_s}")
    return ros_bridge.nfc_bridge.read_nfc_tag(timeout_in_s=timeout_in_s)


"""
======================
QR Reader API Endpoints
======================
"""


@app.get("/read_qr_code", tags=["QR Reader"])
def read_qr_code(expected_data: str, timeout_in_s: int):
    api_logger.info(
        f"Received read QR code request with expected value: {expected_data} and timeout: {timeout_in_s}"
    )
    return ros_bridge.qr_bridge.read_qr_code(
        expected_data=expected_data, timeout_in_s=timeout_in_s
    )


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


@app.get("/disinfection_module_status", tags=["Disinfection"])
def get_disinfection_module_status():
    return ros_bridge.disinfection_module_bridge.get_disinfection_module_status()


@app.post("/refill_disinfection_fluid_container", tags=["Disinfection"])
def refill_disinfection_fluid_container():
    disinfection_module_status = (
        ros_bridge.disinfection_module_bridge.get_disinfection_module_status()
    )
    remaining_disinfections = None
    if disinfection_module_status["status"] == "success":
        remaining_disinfections = disinfection_module_status["remaining_disinfections"]
    api_logger.info(
        f"Received refill disinfection fluid container request at {remaining_disinfections} remaining disinfections"
    )
    return ros_bridge.disinfection_module_bridge.refill_disinfection_fluid_container()


"""
======================
Error API Endpoints
======================
"""


@app.get("/errors", tags=["Errors"])
def get_errors():
    return ros_bridge.error_bridge.get_errors()
