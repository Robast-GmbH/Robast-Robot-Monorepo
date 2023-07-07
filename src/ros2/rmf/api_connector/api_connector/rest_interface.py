from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn


class Drawer(BaseModel):
    id: int
    module_id: int


class Robot(BaseModel):
    fleet_name: str
    name:str


class RestInterface():

    def __init__(self, ros_node, free_fleet_node, polling_interval=0.5, app=FastAPI()):
        self.ros_node = ros_node
        self.free_fleet_node=free_fleet_node
        self.app = app
        self.polling_timer = ros_node.create_timer(polling_interval, self.api_polling_callback)

        
        @self.app.post("/drawer/open")
        def open_drawer(drawer: Drawer, robot:Robot):
            free_fleet_node.handle_open_drawer_request(robot.fleet_name, robot.name, drawer.module_id, drawer.id)
            return 
      
    def run(self, host='0.0.0.0', port=777, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app

    def api_polling_callback(self):
        self.ros_node.get_robot_status()
        self.ros_node.get_drawer_open_status()