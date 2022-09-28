from queue import Empty
from xmlrpc.client import boolean
from sqlalchemy.orm import Session
from sqlalchemy import event, inspect, update
from sqlalchemy.sql.elements import Null
from sqlalchemy.sql.functions import user
import helper

import models
import schemas
import coords_setup


def get_user(db: Session, user_id: int):
    users = db.query(models.User).filter(models.User.id == user_id).first()
    return users


def get_user_by_email(db: Session, email: str):

    return db.query(models.User).filter(models.User.email == email).first()


def get_users(db: Session, skip: int = 0, limit: int = 100):

    return db.query(models.User).offset(skip).limit(limit).all()

def get_users_login(db: Session, userCredentials: schemas.UserLogin):

    return db.query(models.User).filter(models.User.name== userCredentials.name, models.User.hashed_password== userCredentials.hashed_password).first()



def create_user(db: Session, user: schemas.UserCreate):
    fake_hashed_password = user.password
    db_user = models.User(email=user.email, hashed_password=fake_hashed_password, name=user.name)
    db.add(db_user)
    db.commit()
    db.refresh(db_user)
    return db_user

def delete_user(db: Session, user_id: int):
    user = get_user(db = db, user_id= user_id)
    if(user == None):  
        return False
    db.delete(user)
    db.commit()
    return True
    

def get_orders(db: Session) -> object:
    return db.query(models.Order).all()


def create_user_order(db: Session, order: schemas.OrderCreate, user_id: int):
    if(get_user(db=db, user_id=user_id) != None):
        order.goal = coords_setup.convertclientCoords(order.goal)
        db_order = models.Order(**order.dict(), owner_id=user_id)

        db.add(db_order)
        db.commit()
        db.refresh(db_order)
        return db_order
    else:
        return None

@event.listens_for(models.Order, 'init')
@event.listens_for(models.Goal, 'init')
def received_init(target, args, kwargs):

    for rel in inspect(target.__class__).relationships:

        rel_cls = rel.mapper.class_

        if rel.key in kwargs:
            kwargs[rel.key] = rel_cls(**kwargs[rel.key])


#Mapping
def get_roomMap(db: Session):
    return db.query(models.RoomMap).all()


def get_map_by_name(db: Session, name):
    return db.query(models.RoomMap).filter(models.RoomMap.name == name).first()


def create_roomMap(db: Session, rooms_base: schemas.RoomsCreate):
    rooms = []
    for coords in rooms_base.map_yaml["rooms"].items():
        rooms.append(models.Room(x=coords[1]["center point"]["x"], y=coords[1]["center point"]["y"], name=coords[0],
                                  id=coords[0]))
    map_out = models.RoomMap(name=rooms_base.map_name, rooms=rooms)
    db.add(map_out)
    db.commit()
    db.refresh(map_out)
    return map_out

def init(db: Session):
    helper.init(db)
    return

def update_order(db: Session, order_id: int, order: schemas.OrderCreate):
    order_old = db.query(models.Order).filter(models.Order.id == order_id).first()
    
    goal = coords_setup.convertclientCoords(order.goal)

    order_old.order_item=order.order_item
    order_old.order_type=order.order_type
    order_old.recurring_order = order.recurring_order
    order_old.finished = order.finished
    order_old.goal.x = goal.x
    order_old.goal.y = goal.y
    order_old.goal.clientX = goal.clientX
    order_old.goal.clientY = goal.clientY
    order_old.description=order.description

    db.merge(order_old)
    db.commit()
    db.refresh(order_old)
    return order_old


def get_order(db: Session, order_id: int):

    order = db.query(models.Order).filter(models.Order.id == order_id).first()
    if(order == None):
        return None
    return order


def delete_order(db: Session, order_id: int):
    order = get_order(db = db, order_id= order_id)
    user = get_user(db = db, user_id=order.owner_id)
    if(order != None):

        db.delete(order)
        db.commit()
        return True
    else:
        return False   

#drawer


def get_drawers(db: Session) -> object:
    return db.query(models.Drawer).all() 

def get_drawer(db: Session,drawer_controller_id: int):

    drawer = db.query(models.Drawer).filter( models.Drawer.drawer_controller_id==drawer_controller_id).first()
    if(drawer == None):
        return None
    return drawer


def create_drawer(db: Session, drawer: schemas.DrawerCreate):
    db_drawer= get_drawer(db = db,drawer_controller_id = drawer.drawer_controller_id) 
    if(db_drawer != None):
        return None
        
    db_drawer = models.Drawer(drawer_controller_id=drawer.drawer_controller_id, content=drawer.content, empty= drawer.empty)
    db.add(db_drawer)
    db.commit()
    db.refresh(db_drawer)
    return db_drawer


def update_drawer(db: Session,drawer_controller_id: int, content: str ):
    drawer_old = get_drawer(db = db,drawer_controller_id =drawer_controller_id) 
    if(drawer_old== None):
        return None

    drawer_old.content = content
    db.merge(drawer_old)
    db.commit()
    db.refresh(drawer_old)
    return drawer_old


def delete_drawer(db: Session, drawer_controller_id: int):
    drawer = get_drawer(db = db, drawer_controller_id = drawer_controller_id) 
    if(drawer != None):

        db.delete(drawer)
        db.commit()
        return True
    else:
        return False  

def get_empty_drawer(db: Session):
    return db.query(models.Drawer).filter(models.Drawer.empty== True).all()

def change_drawer_empty_state(db: Session, drawer_controller_id:int, empty:bool):
    drawer_old= get_drawer(db=db,drawer_controller_id= drawer_controller_id)
    drawer_old.empty=  empty
    db.merge(drawer_old)
    db.commit()
    db.refresh(drawer_old)
    return 

#Mapposition
def get_map_positions(db: Session) -> object:
    return db.query(models.MapPosition).all() 


def get_map_position(db: Session, id: int):
    map_position = db.query(models.MapPosition).filter(models.MapPosition.id == id).first()
    if(map_position == None):
        return None
    return map_position


def create_map_position(db: Session, map_position: schemas.MapPositionCreate):     
    db_map_position = models.MapPosition(name = map_position.name, x = map_position.x, y = map_position.y, t = map_position.t)
    db.add(db_map_position)
    db.commit()
    db.refresh(db_map_position)
    return db_map_position



def delete_map_position(db: Session, id :int):
    map_position = get_map_position(db = db, id = id) 
    if(map_position != None):
        db.delete(map_position)
        db.commit()
        return True
    else:
        return False    