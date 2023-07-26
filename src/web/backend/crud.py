from queue import Empty
from xmlrpc.client import boolean
from sqlalchemy.orm import Session
from sqlalchemy import event, inspect, update
from sqlalchemy.sql.elements import Null
from sqlalchemy.sql.functions import user
import helper

import models
import schemas


def init(db: Session):
    helper.init(db)
    return

#user_mangement

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
    

# def get_orders(db: Session) -> object:
#     return db.query(models.Order).all()


# def create_user_order(db: Session, order: schemas.OrderCreate, user_id: int):
#     if(get_user(db=db, user_id=user_id) != None):
#         order.goal = coords_setup.convertclientCoords(order.goal)
#         db_order = models.Order(**order.dict(), owner_id=user_id)

#         db.add(db_order)
#         db.commit()
#         db.refresh(db_order)
#         return db_order
#     else:
#         return None

# @event.listens_for(models.Order, 'init')
# @event.listens_for(models.Goal, 'init')
# def received_init(target, args, kwargs):

#     for rel in inspect(target.__class__).relationships:

#         rel_cls = rel.mapper.class_

#         if rel.key in kwargs:
#             kwargs[rel.key] = rel_cls(**kwargs[rel.key])




# #drawer


# def get_drawers(db: Session) -> object:
#     return db.query(models.Drawer).all() 

# def get_drawer(db: Session,drawer_controller_id: int):

#     drawer = db.query(models.Drawer).filter( models.Drawer.drawer_controller_id==drawer_controller_id).first()
#     if(drawer == None):
#         return None
#     return drawer


# def create_drawer(db: Session, drawer: schemas.DrawerCreate):
#     db_drawer= get_drawer(db = db,drawer_controller_id = drawer.drawer_controller_id) 
#     if(db_drawer != None):
#         return None
        
#     db_drawer = models.Drawer(drawer_controller_id=drawer.drawer_controller_id, content=drawer.content, empty= drawer.empty)
#     db.add(db_drawer)
#     db.commit()
#     db.refresh(db_drawer)
#     return db_drawer


# def update_drawer(db: Session,drawer_controller_id: int, content: str ):
#     drawer_old = get_drawer(db = db,drawer_controller_id =drawer_controller_id) 
#     if(drawer_old== None):
#         return None

#     drawer_old.content = content
#     db.merge(drawer_old)
#     db.commit()
#     db.refresh(drawer_old)
#     return drawer_old


# def delete_drawer(db: Session, drawer_controller_id: int):
#     drawer = get_drawer(db = db, drawer_controller_id = drawer_controller_id) 
#     if(drawer != None):

#         db.delete(drawer)
#         db.commit()
#         return True
#     else:
#         return False  

# def get_empty_drawer(db: Session):
#     return db.query(models.Drawer).filter(models.Drawer.empty== True).all()

# def change_drawer_empty_state(db: Session, drawer_controller_id:int, empty:bool):
#     drawer_old= get_drawer(db=db,drawer_controller_id= drawer_controller_id)
#     drawer_old.empty=  empty
#     db.merge(drawer_old)
#     db.commit()
#     db.refresh(drawer_old)
#     return 
