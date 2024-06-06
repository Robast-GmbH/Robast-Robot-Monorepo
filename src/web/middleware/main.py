from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import requests

from task_assignment_system.task_assignment_system import TaskAssignmentSystem
from pydantic_models.module_process_data import ModuleProcessData


app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

name_to_ip = {"rb_theron": "10.10.23.6"}
robot_api_port = 8001
fleet_management_address = "http://10.10.23.6:8000"

task_assigment_system = TaskAssignmentSystem(
    fleet_ip_config=name_to_ip,
    robot_api_port=robot_api_port,
    fleet_management_address=fleet_management_address,
)


def get_robot_url(robot_name: str):
    robot_ip = name_to_ip.get(robot_name, None)
    if robot_ip is None:
        raise HTTPException(status_code=404, detail="Robot name not found")
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
Modules API Endpoints
======================
"""


@app.get("/modules", tags=["Modules"])
def read_modules(robot_url: str = Depends(get_robot_url)):
    response = requests.get(f"{robot_url}/modules").json()
    return response


@app.post("/open_drawer", tags=["Modules"])
def post_open_drawer(
    module_id: int, drawer_id: int, robot_url: str = Depends(get_robot_url)
):
    response = requests.post(
        f"{robot_url}/open_drawer?module_id={module_id}&drawer_id={drawer_id}"
    ).json()
    return response


@app.post("/close_drawer", tags=["Modules"])
def post_close_drawer(
    module_id: int, drawer_id: int, robot_url: str = Depends(get_robot_url)
):
    response = requests.post(
        f"{robot_url}/close_drawer?module_id={module_id}&drawer_id={drawer_id}"
    ).json()
    return response


@app.post("/start_module_process", tags=["Modules"])
def post_start_module_process(
    data: ModuleProcessData,
    robot_url: str = Depends(get_robot_url),
):
    response = requests.post(
        f"{robot_url}/start_module_process",
        json=data.dict(),
    ).json()
    return response


@app.get("/module_process_status", tags=["Modules"])
def get_module_process_state(
    robot_url: str = Depends(get_robot_url),
):
    response = requests.get(f"{robot_url}/module_process_status").json()
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
def post_task_assignment(
    required_drawer_type: int, target_id: str, start_id: str = None
):
    return task_assigment_system.receive_request(
        required_drawer_type, start_id, target_id
    )


@app.get("/robot_tasks", tags=["Tasks"])
def get_robot_tasks(robot_name: str):
    return task_assigment_system.robots[robot_name].get_robot_tasks()


if __name__ == "__main__":
    uvicorn.run(app, port=8003, host="0.0.0.0")
