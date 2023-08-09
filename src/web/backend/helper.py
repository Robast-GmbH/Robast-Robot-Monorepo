from fastapi import Depends
from sqlalchemy.orm import Session
from sqlalchemy import func

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
                addDrawer("RB0", 3, 0, 3, models.DrawerSlideTypes.Manual, 10, db)
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

def json_drawer():
        return { "id":0, "module_id":0, "is_edrawer":False}

def json_robot():
              return { "fleet_name":"", "name":""}
def json_waypoint():
        return {  
                  "pose":{ "x": 0, "y":0, "z": 0 },
                  "orientation" :0
                }
def getDrawer(crud, db:Session, module, robot_name:str, schemas):
        db_module= crud.get_drawer(db=db, module_id=module.id, drawer_id= module.drawer_id,robot_name=robot_name )
        db_robot= crud.get_robot(db=db, robot_name=robot_name)
        if db_module is None:
                return "module"
        
        if db_robot is None:
                return "robot"
        
        robot= json_robot()
        robot["name"]= db_robot.robot_name
        robot["fleet_name"]= db_robot.fleet_name
        drawer= json_drawer()
        drawer["module_id"]=db_module.id
        drawer["id"]= db_module.drawer_id
        drawer["is_edrawer"]= db_module.type == schemas.DrawerSlideTypes.Electrical
        message={ "drawer":drawer, "robot":robot}
        return message
    