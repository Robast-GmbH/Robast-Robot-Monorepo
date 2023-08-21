
import uvicorn
from . import schemas

from fastapi import FastAPI , HTTPException,Body
from typing_extensions import Annotated
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
                if action.type== schemas.ActionType.NAVIGATION:
                    self.ros_node.handle_destination_request( task.robot.fleet_name, task.robot.robot_name, str(task.task_id)+"#"+str(action.step), action.action.pose.x, action.action.pose.y,action.action.yaw)
                elif action.type== schemas.ActionType.OPEN_DRAWER:
                    self.ros_node.handle_slide_drawer_request( task.robot.fleet_name, task.robot.robot_name, str(task.task_id)+"#"+str(action.step), action.action.module_id, action.action.drawer_id, action.action.locked_for, action.action.is_edrawer,True)
                elif action.type== schemas.ActionType.NEW_USER:
                    self.ros_node.handle_new_user_request( task.robot.fleet_name, task.robot.robot_name, str(task.task_id)+"#"+str(action.step), action.action.user_id)
            
        @self.app.post("/settings/move/pause")
        def pause_robot(robot:schemas.Robot):
            self.ros_node.handle_setting_request(robot.robot_name, robot.fleet_name,"move", "pause")
            return
        
        @self.app.post("/settings/move/resume")
        def resume_robot(robot:schemas.Robot):
            self.ros_node.handle_setting_request(robot.robot_name, robot.fleet_name,"move", "resume")
            return

        @self.app.post("/settings/move/cancel")
        def cancel_robot(robot:schemas.Robot ):
            self.ros_node.handle_setting_request(robot.robot_name, robot.fleet_name,"move", "cancel")
            return  
        
        @self.app.post("/settings/drawer/close")
        def close_drawer(robot:schemas.Robot, drawer_id:Annotated[int,Body()], module_id:Annotated[int,Body()], e_drawer:Annotated[bool,Body()]):
            if e_drawer:
                self.ros_node.handle_setting_request( robot.robot_name, robot.fleet_name,"drawer",str(module_id) )
            else:
                    raise HTTPException(status_code=423, detail="drawer has to be closed manually")
        
        @self.app.post("/settings/drawer/completed")
        def end_drawer_action(robot:schemas.Robot):
                self.ros_node.handle_setting_request(robot.robot_name, robot.fleet_name, "drawer", "completed")


    def run(self, host='0.0.0.0', port=3002, log_level='warning'):
        uvicorn.run(self.app, host=host, port=port, log_level=log_level)

    def get_fastapi(self):
        return self.app

    def handle_robot_update(self, robot_name:str,  task_id:str, mode:int, x_pose:float, y_pose:float, yaw_pose:float, battery_level:float):
        task_id= task_id.split('#')[0]
        if not task_id.isnumeric() :
            task_id=0
        else:
            task_id=int(task_id)

        message= self.fill_robot_status_msg(robot_name, "ROBAST", task_id=task_id, x_pose= x_pose, y_pose= y_pose, yaw_pose= yaw_pose, battery_level=battery_level )
        sender = requests.Session()
        sender.post(url= self.response_api+"/robots/status", data= json.dumps(message), verify=False)
 
    def handle_drawer_drawer_status_change(self, task_id, module_id, drawer_id, status):
        #update task
        message= self.fill_action_status(status="Drawer_is"+status, finished=False)
        sender = requests.Session()
        sender.post(url= self.response_api+"/robots/status", data= json.dumps(message), verify=False)

        #update drawer
        message= self.fill_drawer_status(module_id,  drawer_id, status)
        sender = requests.Session()
        sender.post(url= self.response_api+"/robots/modules/status", data= json.dumps(message), verify=False)
    

    def fill_robot_status_msg(self,robot_name:str, fleet_name:str, task_id:int, x_pose:int, y_pose:int, yaw_pose:int, battery_level:float):
        return{ 
                "robot_name": robot_name,
                "fleet_name" : fleet_name,
                "task_id":task_id,
                "x_pose": x_pose,
                "y_pose" :y_pose,
                "yaw_pose": yaw_pose,
                "battery_level":battery_level } 
    
    def fill_action_status(self,status:str, finished:bool):
        return{
                "staus": status,
                "finished":finished

        }
    
    def fill_drawer_status(self, module_id:int,  drawer_id:int, status:str):
        return{
                "module_id":module_id,
                "drawer_id":drawer_id,
                "status": status,
        }
