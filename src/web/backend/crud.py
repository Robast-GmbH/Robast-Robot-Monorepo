from queue import Empty
from xmlrpc.client import boolean
from sqlalchemy.orm import Session
from sqlalchemy import event, inspect, update, asc, desc
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

def create_nfc_code(db:Session, user_id:int, nfc_code:str):
    disable_user_access(db=db, user_id=user_id)
    new_nfc_code= models.Authentication( user_id= user_id, nfc_code=nfc_code) 
    db.add(new_nfc_code)
    db.commit()
    db.refresh(new_nfc_code)
    return new_nfc_code

def get_nfc_codes(db:Session, user_id:int)->[models.Authentication]:
    return db.query(models.Authentication).filter(models.Authentication.id== user_id, models.Authentication.enabled==True).all()

def get_nfc_code(db:Session, user_id)-> str:
    return db.query(models.Authentication.nfc_code).filter(models.Authentication.id==user_id, models.Authentication.enabled==True).first()

def disable_user_access(db:Session, user_id:int):
    old_nfc_codes= get_nfc_codes(db=db,user_id=user_id)
    for code in old_nfc_codes:
        code.enabled= False
    db.flush()
    db.commit() 

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
    return db.query(models.Task).filter().all()

def get_tasks_queue(db:Session):
    subquery=db.query(models.Action.task_id).filter(models.Action.finished== False).distinct()
    return db.query(models.Task).filter(models.Task.id.in_(subquery)).order_by(models.Task.id).all()


def get_task(db:Session, task_id:int): 
    return db.query(models.Task).filter(models.Task.id==task_id).first()


def create_task(db:Session, robot_name:str, fleet_name:str, owner_id :int)->schemas.BaseTask: 
    db_task= models.Task(owner_id= owner_id, robot_name=robot_name, fleet_name=fleet_name)
    db.add(db_task)
    db.commit()
    db.refresh(db_task)
    return db_task.id

def update_task(db:Session, id, robot_name:str, fleet_name:str )->models.Task:
    db_task = get_task(db=db, id = id )
    db_task.robot_name = robot_name
    db_task.fleet_name = fleet_name
    db.flush()
    db.commit()
    db.refresh(db_task)
    
def create_drawer_action(db: Session,step:int, task_id, module_id:int, drawer_id:int):
    db_action= models.DrawerAction(task_id=task_id,finished= False, position=step,drawer_id=drawer_id, module_id=module_id) 
    db.add(db_action)
    db.commit()
    db.refresh(db_action)
    return db_action

def create_move_action(db:Session, step:int, task_id,x:float,y:float,yaw:float):
    db_action= models.MoveAction(task_id=task_id, finished= False, position=step, x_pose=x, y_pose=y, yaw_pose=yaw) 
    db.add(db_action)
    db.commit()
    db.refresh(db_action)
    return db_action

def create_new_user_nfc(db:Session, step:int, task_id, user_id:int ):
    db_action= models.newUserAction(task_id=task_id, finished= False, position=step, user_id= user_id) 
    db.add(db_action)
    db.commit()
    db.refresh(db_action)
    return db_action

def get_actions_of_task(db:Session, task_id:int)->[models.Action]:
    db_actions= db.query(models.Action).filter(models.Action.task_id==task_id).order_by(models.Action.position).all
    return db_actions

def update_action(db:Session, action:schemas.Action )->models.Action:
    db_action= db.query(models.Action).filter(models.Action.id== action.id).first
    db_action.finished= action.finished

def abort_task(db:Session, task_id:int):
    db_task = get_task(db=db, id = task_id )
    
    db.flush()
    db.commit()
    db.refresh(db_task)

#robot
def get_robots(db:Session, skip: int = 0, limit: int = 100)->[models.Robot]:
    db_robots= db.query(models.Robot).offset(skip).limit(limit).all()
    print(db_robots[0].robot_name)
    return db_robots 

def get_robot(db:Session, robot_name:str)-> models.Robot:
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


def get_modules(db: Session, robot_name: str)->[models.Module]:
    return db.query(models.Module).filter(models.Module.robot_name== robot_name).all()

def get_module(db: Session, module_id: int)-> models.Module:
    return db.query(models.Module).filter(models.Module.id == module_id).all()

def get_drawer(db: Session,robot_name:str, module_id: int, drawer_id: int)-> models.Module:
    drawer= db.query(models.Module).filter(models.Module.robot_name ==robot_name, models.Module.id == module_id, models.Module.drawer_id== drawer_id).first()
    return drawer

def set_module(db:Session, module: schemas.Module)-> models.Module:
    db_module= get_drawer(db=db, robot_name= module.robot_name, module_id= module.id, drawer_id = module.drawer_id)
    if(db_module is None):
        db_module = models.Module(id = module.id,
                                drawer_id = module.drawer_id,
                                type = module.type,
                                robot_name = module.robot_name,
                                size= module.size,
                                position= module.position,
                                status= module.status
                                )
        db.add(db_module)
    else:
        db_module.type=module.type
        db_module.robot_name = module.robot_name
        db_module.size= int(module.size)
        db_module.position= module.position
        db_module.status= module.status
    db.commit()
    db.refresh(db_module)
    return db_module

def set_module_status(db:Session,module:schemas.UpdateModule)->models.Module:
    db_module= get_drawer(db=db, robot_name= module.robot_name, module_id= module.id, drawer_id = module.drawer_id)
    if(db_module is None):
        return 
    else:
        db_module.status= module.status
    db.commit()
    db.refresh(db_module)
    return db_module


