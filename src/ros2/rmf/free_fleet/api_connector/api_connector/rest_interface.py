
from pydantic import BaseModel
import uvicorn
from fastapi import FastAPI


class Drawer(BaseModel):
    id: int
    module_id: int
    is_edrawer: bool
    
class OpenDrawer(Drawer):
    locked_for: list[str]

class Robot(BaseModel):
    fleet_name: str
    name: str


class Pose(BaseModel):
    x:  float
    y:  float
    z:  float


class Waypoint(BaseModel):
    pose: Pose
    orientation: float


class RestInterface():

    def __init__(self, ros_node, free_fleet_node, polling_interval=0.5, app=FastAPI()):
        self.ros_node = ros_node
        self.free_fleet_node = free_fleet_node
        self.app = app
        self.polling_timer = self.ros_node.create_timer(polling_interval, self.api_polling_callback)

        @self.app.post("/drawer/open")
        def open_drawer(drawer: OpenDrawer, robot: Robot):
            print("/drawer/open was called")
            free_fleet_node.handle_slide_drawer_request(
                robot.fleet_name,
                robot.name,
                drawer.module_id,
                drawer.id,
                drawer.locked_for,
                drawer.is_edrawer,
                True
                )
            return 
        
        @self.app.post("/drawer/close")
        def close_drawer(drawer: Drawer, robot: Robot):
            self.ros_node.get_logger().info("/drawer/open was called")
            free_fleet_node.handle_slide_drawer_request(
                robot.fleet_name,
                robot.name,
                drawer.module_id,
                drawer.id,
                [],
                drawer.is_edrawer,
                False
                )
            return
        
        @self.app.post("/move")
        def force_move_to_waypoint(waypoint: Waypoint, robot: Robot):
            free_fleet_node.handle_destination_request(robot.fleet_name,
                                                       robot.name,
                                                       waypoint)
            return
        
        
        @self.app.post("/settings/move/pause")
        def pause_robot():
            free_fleet_node.handle_setting_request("move", "pause")
            return
        
        @self.app.post("/settings/move/resume")
        def resume_robot():
            free_fleet_node.handle_setting_request("move", "resume")
            return

        @self.app.post("/settings/move/cancel")
        def cancel_robot():
            free_fleet_node.handle_setting_request("move", "cancel")
            return    

        @self.app.post("/setting/user")
        def create_new_nfc_card(user_id: int):
            free_fleet_node.handle_setting_request("new_user", str(user_id) )
            return

    def run(self, host='0.0.0.0', port=3002, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app

    def api_polling_callback(self):
        self.ros_node.get_robot_status()
        self.ros_node.get_drawer_open_status()
