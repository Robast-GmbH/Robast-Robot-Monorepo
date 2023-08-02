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
    db_user = models.User(email=user.email,
                            hashed_password=fake_hashed_password,
                            name=user.name)
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

#task management
def get_tasks(db: Session):
    return db.query(models.Task).filter(models.Task.finished==False).all()

def get_task(db:Session, task_id:int):
    return db.query(models.Task).filter(models.Task.id==task_id).first()

def create_delivery_task(db:Session, task: schemas.TaskDelivery):
    db_task = models.Task(task_type= "Delivery",
                            x_pose=task.x_pose,
                            y_pose=task.y_pose,
                            yaw_pose=task.yaw_pose,
                            owner_id=task.owner_id,
                            target_id=task.target_id,
                            module_id=task.module_id,
                            drawer_id=task.drawer_id)
    db.add(db_task)
    db.commit()
    db.refresh(db_task)
    return db_task

def create_move_task(db:Session, task: schemas.TaskMove):
    db_task = models.Task(task_type="Move",
                            x_pose=task.x_pose,
                            y_pose=task.y_pose,
                            yaw_pose=task.yaw_pose)
    db.add(db_task)
    db.commit()
    db.refresh(db_task)
    return db_task

def create_drawer_task(db:Session, task: schemas.TaskDrawer):
    db_task = models.Task(task_type="Drawer",
                            owner_id=task.owner_id,
                            target_id=task.target_id,
                            module_id=task.module_id,
                            drawer_id=task.drawer_id)
    db.add(db_task)
    db.commit()
    db.refresh(db_task)
    return db_task

def update_task(db:Session, task_changes: schemas.TaskUpdate):
    db_task = get_task(db=db, id = task_changes.id )
    db_task.robot_name = task_changes.robot
    db_task.status = task_changes.status
    db_task.finished = task_changes.finished

    db.flush()
    db.commit()
    db.refresh(db_task)
    return db_task

def abort_task(db:Session, task_id:int):
    db_task = get_task(db=db, id = task_id )
    db_task.finished= True
    db.flush()
    db.commit()
    db.refresh(db_task)
    return True

#robot
def get_robots(db:Session, skip: int = 0, limit: int = 100):
     return db.query(models.Robot).offset(skip).limit(limit).all()

def get_robot(db:Session, robot_name:str):
     return db.query(models.Robot).filter( models.Robot.robot_name==robot_name).first()

def set_robot(db:Session, robot:schemas.Robot):
    db_robot= get_robot(db=db,robot_name=robot.robot_name)
    if (db_robot is None):
        db_robot = models.Robot(robot_name = robot.robot_name,
                                fleet_name = robot.fleet_name,
                                x_pose = robot.x_pose,
                                y_pose = robot.y_pose,
                                yaw_pose = robot.yaw_pose,
                                task_id = robot.task_id
                                )
        db.add(db_robot)
        db.commit()
        db.refresh(db_robot)
    else:
        db_robot.robot_name = robot.robot_name
        db_robot.fleet_name = robot.fleet_name
        db_robot.x_pose = robot.x_pose
        db_robot.y_pose = robot.y_pose
        db_robot.yaw_pose = robot.yaw_pose
        db_robot.task_id =robot.task_id

        db.flush()
        db.commit()
        db.refresh(db_robot)
        
#Modules 


def get_modules(db: Session, robot_name: str):
    return db.query(models.Module).filter(models.Module.robot== robot_name).all()

def get_module(db: Session, module_id: int):
    return db.query(models.Module).filter(models.Module.id == module_id).all()

def get_drawer(db: Session,robot_name:str, module_id: int, drawer_id: int):
    return db.query(models.Module).filter(models.Module.robot_name ==robot_name, models.Module.id == module_id, models.Module.drawer_id== drawer_id).first()

def set_module(db:Session, module: schemas.Module):
    db_module= get_drawer(db=db, robot_name= module.robot_name, module_id= module.id, drawer_id = module.drawer_id)
    if(db_module is None):
        db_module = models.Module(id = module.id,
                                drawer_id = module.drawer_id,
                                type = module.type,
                                robot_name = module.robot_name,
                                )
        db.add(db_module)
    else:
        db_module.type=module.type
    db.commit()
    db.refresh(db_module)
    return db_module
