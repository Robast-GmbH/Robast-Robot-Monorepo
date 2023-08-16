from fastapi import Depends
from sqlalchemy.orm import Session
from sqlalchemy import func
import schemas
import crud
import models




def init(db: Session):
        
        if(not hasUser(db)):
                addUser( "Werner","Werner Tester", "Robast", False, db )
                addUser( "Torben", "Torben Zurhelle", "Robast2022HH", True, db )
                addUser( "Tobias", "Tobias Alscher", "Robast2022HH", True, db )
                addUser( "Jacob", "Jacob Ritterbach", "Robast2022HH", True, db )
                
                addRobot("RB0", "ROBAST", 0, 0, 0, db)
                addDrawer("RB0", 1, 0, 1, models.DrawerSlideTypes.Manual, 10, db)
                addDrawer("RB0", 2, 0, 2, models.DrawerSlideTypes.Manual, 10, db)
                addDrawer("RB0", 3, 0, 3, models.DrawerSlideTypes.Electrical, 10, db)
                addDrawer("RB0", 4, 0, 4, models.DrawerSlideTypes.Manual, 20, db)
                addDrawer("RB0", 5, 0, 5, models.DrawerSlideTypes.Manual, 30, db)

        return


def addUser( name: str, full_name:str, password:str, admin:bool, db:Session):
        db_user = models.User(email=name+"@Robast.de", hashed_password=password,admin=admin ,name= name, full_name= full_name)
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return

def hasUser (db:Session):
      return db.query(func.count(models.User.id)).scalar() > 0


def addRobot(robot_name:str, fleet_name:str, x:float, y:float, yaw:float, db:Session):
        db_robot = models.Robot(robot_name= robot_name, fleet_name=fleet_name, x_pose=x, y_pose=y, yaw_pose=yaw, task_id=0)
        db.add(db_robot)
        db.commit()
        db.refresh(db_robot)
        return

def addDrawer( robot_name:str, module_id:int, drawer_id:int, position:int, type:models.DrawerSlideTypes, size:int, db:Session):
        db_drawer = models.Module(drawer_id= drawer_id, id= module_id, position= position, status="", type= type, size= size, robot_name= robot_name, )
        db.add(db_drawer)
        db.commit()
        db.refresh(db_drawer)
        return

def addMapPosition( name: str, x: float, y: float, t:float, db:Session):
        db_map_position = models.MapPosition( name= name, x=x, y=y, t=t)
        db.add(db_map_position)
        db.commit()
        db.refresh(db_map_position)
        return

def json_drawer(id:int, module_id:int, is_edrawer:bool, nfc_codes:list[str])-> dict:
        return { "id": id, "module_id": module_id, "is_edrawer":is_edrawer, "locked_for":nfc_codes}

def json_robot(fleet_name:str,robot_name:str)->dict:
              return { "fleet_name": fleet_name, "name": robot_name}

def json_waypoint(x:float,y:float,z:float,yaw:float)-> dict:
        return {  
                  "pose":{ "x": 0, "y":0, "z": 0 },
                  "orientation" :0
                }

def find_robot_json(db:Session, robot_name):
        db_robot= crud.get_robot(db=db, robot_name=robot_name)
        if db_robot is None:
                return "robot"
        return json_robot(robot_name=db_robot.fleet_name, fleet_name=db_robot.fleet_name)

def find_drawer_json(db:Session,module_id:int,drawer_id:int , robot_name:str, nfc_codes:list[str]):
        db_module= crud.get_drawer(db=db, module_id= module_id, drawer_id= drawer_id, robot_name=robot_name )

        if db_module is None:
                return "module"
        return json_drawer(module_id=db_module.id, id=db_module.drawer_id, is_edrawer= (db_module.type == schemas.DrawerSlideTypes.Electrical), nfc_codes=nfc_codes)

def get_Drawer_interaction_json(db:Session, module, robot_name:str, nfc_codes:list[str]):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        
        drawer= find_drawer_json(db, module.id, module.drawer_id, robot_name= robot_name,nfc_codes= nfc_codes)
        if(drawer=="drawer"):
                return drawer
        
        
        message={"drawer":drawer, "robot":robot}
        return message


def get_close_drawer_interaction_json(db:Session, robot_name:str, drawer_id:int,module_id:int, e_drawer:bool):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        
        drawer= find_drawer_json(db,module_id,drawer_id, robot_name)
        if(drawer=="drawer"):
                return drawer
        
        
        message={"robot":robot, "drawer_id":drawer_id, "module_id":module_id, "e:drawer":e_drawer}
        return message

def create_task(db:Session, robot_name:str,task_id:int, action:list):
        robot= find_robot_json(db, robot_name)
        if(robot=="robot"):
                return robot
        message={"robot":robot, "task_id": task_id, "action": action}
        return message

def create_drawer_action(step:int, drawer_id:int, module_id:int, is_edrawer:bool, locked_for:list[str]):
        message={"step":step,"type": "OPEN_DRAWER","action":{"id":drawer_id, "module_id":module_id, "is_edrawer":is_edrawer,"locked_for":locked_for}}
        return message

def create_move_action(step:int, x:float, y:float, yaw:float):
        message={"step":step, "type":"MOVE","action":{ "pose": {"x":x, "y":y, "z":0.0 },"orientation": yaw} }
        return message

def create_new_user(step:int, user_id: int):
        message={"step":step,"type":"NEW_USER", "user_id": user_id}
        return message