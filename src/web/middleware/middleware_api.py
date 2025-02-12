from fastapi import FastAPI, Depends, Response
from fastapi.middleware.cors import CORSMiddleware
import requests


from user_system.user_system_router import user_system_router
from module_manager.module_manager_router import module_manager_router
from manuals_manager.manuals_router import manuals_router
from task_system.task_system_router import task_system_router
from hygiene_manager.hygiene_router import hygiene_router
from fire_alarm.fire_alarm_router import fire_alarm_router
from log_manager.log_router import log_router
import configs.url_config as url_config
from models.url_helper import URLHelper

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(user_system_router, prefix="/users")
app.include_router(module_manager_router, prefix="/modules")
app.include_router(manuals_router, prefix="/manuals")
app.include_router(task_system_router, prefix="/tasks")
app.include_router(hygiene_router, prefix="/hygiene")
app.include_router(fire_alarm_router, prefix="/fire_alarm")
app.include_router(log_router, prefix="/logs")


name_to_ip = url_config.ROBOT_NAME_TO_IP
fleet_management_address = url_config.FLEET_MANAGEMENT_ADDRESS

get_robot_url = URLHelper.get_robot_url


"""
======================
General API Endpoints
======================
"""


@app.get("/")
def read_root():
    return "API is running"


@app.get("/fleet")
def read_fleet():
    return {"fleet": list(name_to_ip.keys())}


"""
===========================
Robot Status API Endpoints
===========================
"""


@app.get("/is_ros_bridge_connected", tags=["Robot Status"])
def read_is_ros_bridge_connected(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/is_ros_bridge_connected").json()
    return response


@app.get("/robot_pos", tags=["Robot Status"])
def read_robot_pos(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/robot_pos").json()
    return response


@app.get("/battery_status", tags=["Robot Status"])
def read_robot_battery_status(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/battery_status").json()
    return response


@app.get("/disinfection_triggered", tags=["Robot Status"])
def read_disinfection_triggered(
    robot_url: str = Depends(get_robot_url), timeout: int = 10
):
    response = requests.get(
        f"{robot_url}/disinfection_triggered?time_out={timeout}"
    ).json()
    return response


@app.get("/disinfection_module_status", tags=["Robot Status"])
def read_disinfection_module_status(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/disinfection_module_status").json()
    return response


"""
=========================
Navigation API Endpoints
=========================
"""


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(
    x: float,
    y: float,
    z: float,
    use_reorientation: bool = False,
    robot_url: str = Depends(get_robot_url),
):
    response = requests.post(
        f"{robot_url}/goal_pose?x={x}&y={y}&z={z}&use_reorientation={use_reorientation}"
    ).content
    return response


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal(robot_url: str = Depends(get_robot_url)):
    response = requests.post(f"{robot_url}/cancel_goal").content
    return response


@app.get("/is_navigation_completed", tags=["Navigation"])
def read_is_navigating(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/is_navigation_completed").json()
    return response


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/remaining_nav_time").json()
    return response


@app.post("/block_navigation", tags=["Navigation"])
def post_block_nav(robot_url: str = Depends(get_robot_url)):
    response = requests.post(f"{robot_url}/block_navigation").json()
    return response


@app.post("/unblock_navigation", tags=["Navigation"])
def post_unblock_nav(robot_url: str = Depends(get_robot_url)):
    response = requests.post(f"{robot_url}/unblock_navigation").json()
    return response


@app.get("/is_navigation_blocked", tags=["Navigation"])
def read_is_nav_blocked(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/is_navigation_blocked").json()
    return response


@app.get("/requires_replan", tags=["Navigation"])
def read_requires_replan(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/requires_replan").json()
    return response


"""
======================
Doors API Endpoints
======================
"""


@app.get("/open_door", tags=["Doors"])
def get_open_door(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/open_door").json()
    return response


@app.get("/close_door", tags=["Doors"])
def get_close_door(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/close_door").json()
    return response


"""
======================
RMF API Endpoints
======================
"""


@app.get("/building_map", tags=["RMF"])
def get_building_map():
    response = requests.get(f"{fleet_management_address}/building_map").json()
    return response


@app.get("/building_map.png", tags=["RMF"])
def get_building_map_png():
    response = requests.get(f"{fleet_management_address}/building_map").json()
    image_url = response["levels"][0]["images"][0]["data"]
    index = image_url.find("/", image_url.find("//") + 2)
    response = requests.get(f"{fleet_management_address}{image_url[index:]}")
    return Response(content=response.content, media_type="image/png")


"""
======================
NFC API Endpoints
======================
"""

@app.get("/read_nfc_tag", tags=["NFC"])
def read_nfc_tag(timeout_in_s: int = 30, robot_url: str = Depends(get_robot_url)):
    response = requests.get(
        f"{robot_url}/read_nfc_tag?timeout_in_s={timeout_in_s}"
    ).json()
    return response
