from fastapi import Depends
from sqlalchemy.orm import Session
from sqlalchemy import func

import models

next_point = 1
robot_status = 2
open_drawer = -1


def init(db: Session):
        
        if(not hasUser(db)):
                addUser( "Werner","Werner Tester", "Robast", False, db )
                addUser( "Torben", "Torben Zurhelle", "Robast2022HH", True, db )
                addUser( "Tobias", "Tobias Alscher", "Robast2022HH", True, db )
                addUser( "Jacob", "Jacob Ritterbach", "Robast2022HH", True, db )

        return

def addUser( name: str, full_name:str, password:str, admin:bool, db:Session):
        db_user = models.User(email=name+"@Robast.de", hashed_password=password,admin=admin ,name= name, full_name= full_name)
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return

def hasUser (db:Session):
      return db.query(func.count(models.User.id)).scalar() > 0

def addDrawer( content: str, drawer_controller_id:int, empty:bool, db:Session):
        db_drawer = models.Drawer( drawer_controller_id=drawer_controller_id, content=content, empty=empty)
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
        return { "id":0, "module_id":0}

def json_robot():
              return { "fleet_name":"", "name":""}
def json_waypoint():
        return {  
                  "pose":{ "x": 0, "y":0, "z": 0 },
                  "orientation" :0
                }