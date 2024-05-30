from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import requests


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


def get_robot_ip(robot_name: str):
    robot_ip = name_to_ip.get(robot_name, None)
    if robot_ip is None:
        raise HTTPException(status_code=404, detail="Robot name not found")
    return robot_ip


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
def read_is_ros_bridge_connected(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(
        f"http://{robot_ip}:{robot_api_port}/is_ros_bridge_connected"
    ).json()
    return response


@app.get("/robot_pos", tags=["Robot Status"])
def read_robot_pos(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(f"http://{robot_ip}:{robot_api_port}/robot_pos").json()
    return response


@app.get("/battery_level", tags=["Robot Status"])
def read_robot_battery_level(robot_ip: str = Depends(get_robot_ip)):
    return {"battery_level": 1.0}


"""
=========================
Navigation API Endpoints
=========================
"""


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(x: float, y: float, z: float, robot_ip: str = Depends(get_robot_ip)):
    response = requests.post(
        f"http://{robot_ip}:{robot_api_port}/goal_pose?x={x}&y={y}&z={z}"
    ).content
    return response


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal(robot_ip: str = Depends(get_robot_ip)):
    response = requests.post(f"http://{robot_ip}:{robot_api_port}/cancel_goal").content
    return response


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(f"http://{robot_ip}:{robot_api_port}/is_navigating").json()
    return response


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(
        f"http://{robot_ip}:{robot_api_port}/remaining_nav_time"
    ).json()
    return response


@app.post("/block_navigation", tags=["Navigation"])
def post_block_nav(robot_ip: str = Depends(get_robot_ip)):
    response = requests.post(
        f"http://{robot_ip}:{robot_api_port}/block_navigation"
    ).json()
    return response


@app.post("/unblock_navigation", tags=["Navigation"])
def post_unblock_nav(robot_ip: str = Depends(get_robot_ip)):
    response = requests.post(
        f"http://{robot_ip}:{robot_api_port}/unblock_navigation"
    ).json()
    return response


@app.get("/is_navigation_blocked", tags=["Navigation"])
def read_is_nav_blocked(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(
        f"http://{robot_ip}:{robot_api_port}/is_navigation_blocked"
    ).json()
    return response


"""
======================
Modules API Endpoints
======================
"""


@app.get("/modules", tags=["Modules"])
def read_modules(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(f"http://{robot_ip}:{robot_api_port}/modules").json()
    return response


@app.post("/open_drawer", tags=["Modules"])
def post_open_drawer(
    module_id: int, drawer_id: int, robot_ip: str = Depends(get_robot_ip)
):
    response = requests.post(
        f"http://{robot_ip}:{robot_api_port}/open_drawer?module_id={module_id}&drawer_id={drawer_id}"
    ).json()
    return response


@app.post("/close_drawer", tags=["Modules"])
def post_close_drawer(
    module_id: int, drawer_id: int, robot_ip: str = Depends(get_robot_ip)
):
    response = requests.post(
        f"http://{robot_ip}:{robot_api_port}/close_drawer?module_id={module_id}&drawer_id={drawer_id}"
    ).json()
    return response


"""
======================
Doors API Endpoints
======================
"""


@app.get("/open_door", tags=["Doors"])
def get_open_door(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(f"http://{robot_ip}:{robot_api_port}/open_door").json()
    return response


@app.get("/close_door", tags=["Doors"])
def get_close_door(robot_ip: str = Depends(get_robot_ip)):
    response = requests.get(f"http://{robot_ip}:{robot_api_port}/close_door").json()
    return response


if __name__ == "__main__":
    uvicorn.run(app, port=8003, host="0.0.0.0")
