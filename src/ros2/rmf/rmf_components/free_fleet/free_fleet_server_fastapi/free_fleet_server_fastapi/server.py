
from typing import List
from fastapi import FastAPI
import uvicorn
from free_fleet_server_fastapi.freefleet import free_fleet_server
import free_fleet_server_fastapi.fastapi_classes as fastapi_classes





app = FastAPI()

@app.get("/robot")
def show_robot_status():
    return free_fleet_server.get_robot_states()

@app.get("/{fleet_name}/{robot_name}/status")
def read_item(fleet_name: str, robot_name: str):
    return free_fleet_server.get_robot_states(fleet_name, robot_name)

@app.put("/{fleet_name}/{robot_name}/mode")
def change_mode(fleet_name: str,robot_name: str, mode :str):
    free_fleet_server.handle_mode_request(fleet_name, robot_name, mode)

@app.put("/{fleet_name}/{robot_name}/path")
def change_path(fleet_name: str, robot_name: str, path: List[fastapi_classes.Location] ):
    free_fleet_server.handle_path_request(fleet_name, robot_name, path)

@app.put("/{fleet_name}/{robot_name}/destination")
def change_destination(fleet_name: str, robot_name: str, destination: fastapi_classes.Location):
    free_fleet_server.handle_destination_request(fleet_name, robot_name, destination)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)