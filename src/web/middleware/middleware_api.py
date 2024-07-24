from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import requests

from task_assignment_system.task_assignment_system import TaskAssignmentSystem
from pydantic_models.delivery_request import DeliveryRequest
from user_system.user_system_router import user_system_router
from module_manager.module_manager_router import module_manager_router
import configs.url_config as url_config


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


name_to_ip = url_config.ROBOT_NAME_TO_IP
robot_api_port = url_config.ROBOT_API_PORT
fleet_management_address = url_config.FLEET_MANAGEMENT_ADDRESS


task_assigment_system = TaskAssignmentSystem()


def get_robot_url(robot_name: str):
    robot_ip = name_to_ip.get(robot_name, None)
    if robot_ip is None:
        raise HTTPException(404, detail="Robot name not found")
    return f"http://{robot_ip}:{robot_api_port}"


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


@app.get("/battery_level", tags=["Robot Status"])
def read_robot_battery_level(robot_url: str = Depends(get_robot_url)):
    return {"battery_level": 1.0}


"""
=========================
Navigation API Endpoints
=========================
"""


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(
    x: float, y: float, z: float, robot_url: str = Depends(get_robot_url)
):
    response = requests.post(f"{robot_url}/goal_pose?x={x}&y={y}&z={z}").content
    return response


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal(robot_url: str = Depends(get_robot_url)):
    response = requests.post(f"{robot_url}/cancel_goal").content
    return response


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/is_navigating").json()
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
Tasks API Endpoints
======================
"""


@app.post("/task_assignment", tags=["Tasks"])
def post_task_assignment(request: DeliveryRequest):
    success, message = task_assigment_system.receive_request(request)
    return {"success": success, "message": message}


@app.get("/tasks", tags=["Tasks"])
def get_tasks():
    tasks = {}
    for robot in task_assigment_system.robots.values():
        tasks[robot.name] = robot.get_robot_tasks()
    return tasks


@app.get("/robot_tasks", tags=["Tasks"])
def get_robot_tasks(robot_name: str):
    return task_assigment_system.robots[robot_name].get_robot_tasks()


"""
======================
RMF API Endpoints
======================
"""


@app.get("/building_map", tags=["RMF"])
def get_building_map():
    response = requests.get(f"{fleet_management_address}/building_map").json()
    return response
