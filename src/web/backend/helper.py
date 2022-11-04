from fastapi import Depends
from sqlalchemy.orm import Session
import models

next_point = 1
robot_status = 2
open_drawer = -1


def init(db: Session):
        addUser( "Werner","Werner Tester", "Robast", False, db )
        addUser( "Torben", "Torben Zurhelle", "Robast2022HH", True, db )
        addUser( "Tobias", "Tobias Alscher", "Robast2022HH", True, db )
        addUser( "Jacob", "Jacob Ritterbach", "Robast2022HH", True, db )

        addDrawer( "Schublade 1", 1, True , db)
        addDrawer( "Schublade 2", 2, True , db)
        addDrawer( "Schublade 3", 3, True , db)
        addDrawer( "Schublade 4", 4, True , db)
        addDrawer( "Schublade 5", 5, True , db)

       #addMapPosition( "home", 0.0, 0.0, 0.0, db)

        return

def addUser( name: str, full_name:str, password:str, admin:bool, db:Session):
        db_user = models.User(email=name+"@Robast.de", hashed_password=password,admin=admin ,name= name, full_name= full_name)
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return

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