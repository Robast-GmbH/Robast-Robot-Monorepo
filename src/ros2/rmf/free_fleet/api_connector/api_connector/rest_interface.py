
import uvicorn
from . import schemas

from fastapi import FastAPI , HTTPException
import requests
import json

class RestInterface():

    def __init__(self, ros_node, app:FastAPI =FastAPI() ):
        self.ros_node = ros_node
        ros_node.set_responce(self)
        self.app = app
        self.config= ros_node.get_dds_config()
        self.response_api=self.config["backend_address"]
        
        @self.app.post("/task")
        def do_task( task:schemas.Task):
            for action in task.actions:
                if action.type=="move":
                    self.ros_node.handle_destination_request( task.robot.fleet_name, task.robot.robot_name, task.task_id+"#"+action.step, action.waypoint.pose.x, action.waypoint.y,action.orientation)
                elif action.type=="drawer":
                    self.ros_node.handle_slide_drawer_request( task.robot.fleet_name, task.robot.robot_name, task.task_id+"#"+action.step, action.drawer.module_id, action.drawer.drawer_id, action.drawer.locked_for, action.drawer.is_edrawer, action.drawer.open)
                elif action.type=="NFC_generation":
                    self.ros_node.handle_new_user_request( task.robot.fleet_name, task.robot.robot_name, task.task_id+"#"+action.step, action.new_user.user_id)
            
        @self.app.post("/settings/move/pause")
        def pause_robot(robot:schemas.Robot):
            self.ros_node.handle_setting_request("move", "pause")
            return
        
        @self.app.post("/settings/move/resume")
        def resume_robot(robot:schemas.Robot):
            self.ros_node.handle_setting_request("move", "resume")
            return

        @self.app.post("/settings/move/cancel")
        def cancel_robot(robot:schemas.Robot ):
            self.ros_node.handle_setting_request("move", "cancel")
            return  
        
        @self.app.post("/settings/drawer/close")
        def close_drawer(robot:schemas.Robot, drawer_id:int, module_id:int, e_drawer:bool):
            if e_drawer:
                self.ros_node.handle_setting_request("drawer", str(module_id))
            else:
                    raise HTTPException(status_code=404, detail="drawer not found")
        
        @self.app.post("/settings/drawer/completed")
        def end_drawer_action(robot:schemas.Robot):
                self.ros_node.handle_setting_request("drawer", "completed")


    def run(self, host='0.0.0.0', port=3002, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app

    def handle_robot_update(self, robot_name:str,  task_id:str, mode:int, x_pose:float, y_pose:float, yaw_pose:float):
        #update position
        message=self.fill_robot_status_msg(robot_name, "ROBAST", task_id=task_id, x_pose= x_pose, y_pose= y_pose, yaw_pose= yaw_pose )
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session()
        sender.post(url= self.response_api+"/robots/status", json= json.dumps(message), headers= headers, verify=False)
      
        # update robot Mode
        #if idle remove task from robot  
        #and request new task

    
    def handle_task_update(self, task_id:str, step:int, status:str, status_msg:str, finished:bool):
        # update action status 
        message= self.fill_task_status_msg( status, status_msg, finished)
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session()
        sender.post(url= self.response_api+"/robots/"+task_id+"/"+step+"status", json= json.dumps(message), headers= headers, verify=False)

    def fill_robot_status_msg(self,robot_name:str, fleet_name:str, task_id:str, x_pose:int, y_pose:int, yaw_pose:int):
        return{ 
                "robot_name": robot_name,
                "fleet_name" : fleet_name,
                "task_id":task_id,
                "x_pose": x_pose,
                "y_pose" :y_pose,
                "yaw_pose": yaw_pose } 
    
    def fill_task_status_msg(self, status:str, status_msg:str, finished:bool):
        return{
                "status": status,
                "status_msg": status_msg,
                "finished": finished}
