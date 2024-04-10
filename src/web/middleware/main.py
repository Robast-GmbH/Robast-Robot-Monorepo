from fastapi import FastAPI
import uvicorn
import requests

app = FastAPI()

name_to_ip = {"rb_theron":"localhost"}
robot_api_port = 8001


@app.get("/")
def read_root():
    return "API is running"

@app.get("/fleet")
def read_fleet():
    return {"fleet":list(name_to_ip.keys())}


@app.get("/is_ros_bridge_connected")
def read_is_ros_bridge_connected(robot_name: str):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.get(f"http://{robot_ip}:{robot_api_port}/is_ros_bridge_connected").json()
    return response


@app.get("/robot_pos", tags=["Robot Position"])
def read_robot_pos(robot_name: str):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.get(f"http://{robot_ip}:{robot_api_port}/robot_pos").json()
    return response


@app.post("/goal_pose", tags=["Navigation"])
def post_goal_pose(robot_name: str,x: float, y: float, z: float):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.post(f"http://{robot_ip}:{robot_api_port}/goal_pose?x={x}&y={y}&z={z}").content
    return response


@app.post("/cancel_goal", tags=["Navigation"])
def post_cancel_goal(robot_name: str):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.get(f"http://{robot_ip}:{robot_api_port}/cancel_goal").content
    return response


@app.get("/is_navigating", tags=["Navigation"])
def read_is_navigating(robot_name: str):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.get(f"http://{robot_ip}:{robot_api_port}/is_navigating").json()
    return response


@app.get("/remaining_nav_time", tags=["Navigation"])
def read_remaining_nav_time(robot_name: str):
    robot_ip = name_to_ip.get(robot_name,None)
    if robot_ip is None:
        return {"error":"robot name not found"}

    response = requests.get(f"http://{robot_ip}:{robot_api_port}/remaining_nav_time").json()
    return response
    
uvicorn.run(app, port=8003)