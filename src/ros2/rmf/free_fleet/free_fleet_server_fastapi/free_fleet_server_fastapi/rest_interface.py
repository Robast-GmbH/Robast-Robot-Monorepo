
from typing import List
import uvicorn
import free_fleet_server_fastapi.fastapi_classes as fastapi_classes
from fastapi import FastAPI, HTTPException



class RestInterface():

    def __init__(self, ros_node, app=FastAPI()):
        self.ros_node = ros_node
        self.app = app


        #server=free_fleet_server()

        @self.app.get("/robot")
        def show_robot_status():
            return self.ros_node.get_robot_states()

        @self.app.get("/{fleet_name}/{robot_name}/status")
        def read_item(fleet_name: str, robot_name: str):
             status = self.ros_node.get_robot_status(robot_name)
             if len(status) != 1:
                raise HTTPException(status_code=404, detail="Item not found")
             else:
                 return  status[0]

        @self.app.put("/{fleet_name}/{robot_name}/mode")
        def change_mode(fleet_name: str,robot_name: str, mode :str):
            self.ros_node.handle_mode_request(fleet_name, robot_name, mode)

        @self.app.put("/{fleet_name}/{robot_name}/path")
        def change_path(fleet_name: str, robot_name: str, path: List[fastapi_classes.Location] ):
            self.ros_node.handle_path_request(fleet_name, robot_name, path)

        @self.app.put("/{fleet_name}/{robot_name}/destination")
        def change_destination(fleet_name: str, robot_name: str, destination: fastapi_classes.Location):
            self.ros_node.handle_destination_request(fleet_name, robot_name, destination)

    def run(self, host='0.0.0.0', port=8000, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app
