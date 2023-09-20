from fastapi import Depends
from sqlalchemy.orm import Session
from sqlalchemy import func
import schemas
import crud
import models




def init(db: Session):
           
        addUser( "Werner","Werner Tester", "Robast2022HH", False, db )
        addUser( "Torben", "Torben Zurhelle", "Robast2022HH", True, db )
        addUser( "Tobias", "Tobias Alscher", "Robast2022HH", True, db )
        addUser( "Jacob", "Jacob Ritterbach", "Robast2022HH", True, db )
                
        addRobot("RB0", "ROBAST", 0, 0, 0, db)
        addDrawer("RB0", 1, 0, 1, models.DrawerSlideTypes.Manual, 10,  db)
        addDrawer("RB0", 2, 0, 2, models.DrawerSlideTypes.Manual, 10, db)
        addDrawer("RB0", 3, 0, 3, models.DrawerSlideTypes.Electrical, 10, db)
        addDrawer("RB0", 4, 0, 4, models.DrawerSlideTypes.Manual, 20, db)
        addDrawer("RB0", 5, 0, 5, models.DrawerSlideTypes.Manual, 30, db)
        return

def addUser( name: str, full_name:str, password:str, admin:bool, db:Session):
        user=schemas.UserCreate(name=name, full_name=full_name, email=name+"@Robast.de", hashed_password=password, admin=admin)
        return crud.set_user(db=db, override=False, user=user)

def addRobot(robot_name:str, fleet_name:str, x:float, y:float, yaw:float, db:Session):
        robot = schemas.RobotStatus(robot_name=robot_name, fleet_name=fleet_name, x_pose=x, y_pose=y, yaw_pose=yaw, battery_level=0, task_id=None )
        return crud.set_robot(db=db, override=False, robot=robot)
        

def addDrawer( robot_name:str, module_id:int, drawer_id:int, position:int, type:models.DrawerSlideTypes, size:int, db:Session):
        module= schemas.Module(drawer_id=drawer_id, module_id=module_id, robot_name=robot_name, status="Closed", position=position, type=type, size=size, label="")
        return crud.set_module(db=db, override=False, module= module)

def addMapPosition( name: str, x: float, y: float, t:float, db:Session):
        db_map_position = models.MapPosition( name= name, x=x, y=y, t=t)
        db.add(db_map_position)
        db.commit()
        db.refresh(db_map_position)
        return

def json_drawer(id:int, module_id:int, is_edrawer:bool, nfc_codes:list[str])-> dict:
        return { "id": id, "module_id": module_id, "is_edrawer":is_edrawer, "locked_for":nfc_codes}

def json_robot(fleet_name:str,robot_name:str)->dict:
              return { "fleet_name": fleet_name, "robot_name": robot_name}

def json_waypoint(x:float, y:float, z:float, yaw:float)-> dict:
        return {  
                  "pose":{ "x": 0, "y":0, "z": 0 },
                  "yaw" :0
                }

def find_robot_json(db:Session, robot_name):
        db_robot= crud.get_robot(db=db, robot_name=robot_name)
        if db_robot is None:
                return "robot"
        return json_robot(robot_name=db_robot.robot_name, fleet_name=db_robot.fleet_name)

def find_drawer_json(db:Session,module_id:int,drawer_id:int , robot_name:str, nfc_codes:list[str]):
        db_module= crud.get_drawer(db=db, module_id= module_id, drawer_id= drawer_id, robot_name=robot_name )

        if db_module is None:
                return "module"
        return json_drawer(module_id=db_module.module_id, id=db_module.drawer_id, is_edrawer= (db_module.type == schemas.DrawerSlideTypes.Electrical), nfc_codes=nfc_codes)

def get_Drawer_interaction_json(db:Session, module, robot_name:str, nfc_codes:list[str]):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        
        drawer= find_drawer_json(db, module.module_id, module.drawer_id, robot_name= robot_name,nfc_codes= nfc_codes)
        if(drawer=="drawer"):
                return drawer
        
        
        message={"drawer":drawer, "robot":robot}
        return message


def get_close_drawer_interaction_json(db:Session, robot_name:str, drawer_id:int,module_id:int, e_drawer:bool):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        
        drawer= find_drawer_json(db,module_id, drawer_id, robot_name,[])
        if(drawer=="drawer"):
                return drawer
        
        
        message={"robot":robot, "drawer_id":drawer_id, "module_id":module_id, "e_drawer":e_drawer}
        return message

def create_task(db:Session, robot_name:str,task_id:int, action:list):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        message={"robot":robot, "task_id": str(task_id), "actions": action}
        return message

def create_action_list(db:Session, robot_name:str, actions:list[schemas.Action]):
        action_list=[]
        for action in actions:
                if(action.type == schemas.ActionType.DRAWER):
                        db_module=crud.get_drawer(db=db, robot_name=robot_name, module_id=action.action.module_id, drawer_id=action.action.drawer_id)
                        action_list.append(create_drawer_action(step= action.step, drawer_id= action.action.drawer_id, finished=action.finished, module_id= action.action.module_id, is_edrawer= db_module.type==schemas.DrawerSlideTypes.Electrical, locked_for= action.action.locked_for))

                elif(action.type == schemas.ActionType.NAVIGATION):
                        action_list.append(create_navigation_action(action.step, action.action.pose.x,action.action.pose.y,action.action.yaw, action.finished))
                elif(action.type == schemas.ActionType.NEW_USER):  
                        action_list.append(create_new_user(action.step, action.action.user_id, action.finished))   
        return action_list   

def create_drawer_action(step:int, drawer_id:int, module_id:int, is_edrawer:bool, locked_for:list[int], finished:bool=False):
        message={"step":step,"type": "OPEN_DRAWER", "finished":finished, "action":{"drawer_id":drawer_id, "module_id":module_id, "is_edrawer":is_edrawer,"locked_for": locked_for}}
        return message

def create_navigation_action(step:int, x:float, y:float, yaw:float, finished:bool=False):
        message={"step":step, "type":"NAVIGATION", "finished":finished, "action":{ "pose": {"x":x, "y":y, "z":0.0 },"yaw": yaw} }
        return message

def create_new_user(step:int, user_id: int, finished:bool=False):
        message={"step":step, "type":"NEW_USER", "finished":finished, "user_id": user_id}
        return message
