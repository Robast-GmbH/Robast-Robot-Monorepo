from queue import Empty
from xmlrpc.client import boolean
from sqlalchemy.orm import Session
from sqlalchemy import event, inspect, update, asc, desc, distinct,cast
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


def tasks_queue(db:Session):
    subquery= db.query(models.Action.task_id).filter(models.Action.finished==False).all()
    active_tasks= list(map(lambda x:x[0], subquery))
    queue= db.query(models.Task).filter(models.Task.id.in_(active_tasks)).order_by(models.Task.id).all()
    return queue 


def get_task(db:Session, task_id:int): 
    return db.query(models.Task).filter(models.Task.id==task_id).first()


def create_task(db:Session, robot_name:str, fleet_name:str, owner_id :int)->int: 
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

def create_base_action(db:Session, task_id, step):
    db_action= models.Action(task_id=task_id, step=step, type= schemas.ActionType.DRAWER) 
    db.add(db_action)
    db.commit()
    db.refresh(db_action)

    return db_action

def create_drawer_action(db: Session,step:int, task_id, module_id:int, drawer_id:int, owner:int):
    db_action=create_base_action(db, task_id, step)
   
    db_drawer_action=models.DrawerAction( id=db_action.id, target_user_id= owner, drawer_id= drawer_id, module_id= module_id)
    db.add(db_drawer_action)
    db.commit()
    db.refresh(db_drawer_action)
    return

def create_navigation_action(db:Session, step:int, task_id,x:float,y:float,yaw:float):
    db_action=create_base_action(db, task_id, step)

    db_nav_action= models.NavigationAction( id=db_action.id, x_pose= x, y_pose=y, yaw_pose=yaw) 
    db.add(db_nav_action)
    db.commit()
    db.refresh(db_nav_action)
    return db_nav_action

def create_new_user_nfc(db:Session, step:int, task_id, user_id:int ):
    db_action=create_base_action(db, task_id, step)

    db_new_user_action= models.NewUserAction( id=db_action.id, user_id= user_id) 
    db.add(db_new_user_action)
    db.commit()
    db.refresh(db_new_user_action)
    return db_new_user_action

def get_actions(task_id:str, db:Session) ->[models.Action]:
    return db.query(models.Action).filter(models.Action.task_id==task_id).all()

def get_action(action_id:int, db:Session)->models.Action:
    return db.query(models.Action).filter(models.Action.id == action_id).first()

def get_navigation_full_action(action_id:int, db:Session):
    return db.query(models.Action, models.NavigationAction).join(models.Action.id==models.NavigationAction.id).filter(models.NavigationAction.id==action_id).first()

def get_drawer_full_action(action_id:int, db:Session):
    return db.query(models.Action, models.DrawerAction).join(models.Action.id==models.DrawerAction.id).filter(models.DrawerAction.id==action_id).first()

def get_user_full_action(action_id:int, db:Session):
    return db.query(models.Action, models.NewUserAction).join(models.Action.id==models.NewUserAction.id).filter(models.NewUserAction.id==action_id).first()

def get_navigation_action(action_id:int, db:Session):
    return db.query(models.NavigationAction).filter(models.NavigationAction.id==action_id).first()

def get_drawer_action(action_id:int, db:Session):
    return db.query( models.DrawerAction).filter(models.DrawerAction.id==action_id).first()

def get_user_action(action_id:int, db:Session):
    return db.query(models.NewUserAction).filter(models.NewUserAction.id==action_id).first()

def get_action_id(db:Session, task_id:str, step:int):
    return db.query(models.Action.id).filter(models.Action.task_id== task_id and models.Action.step==step).first()[0]
    

def update_action(db:Session, action_id:int, status:str, finished:bool):
    db_action = get_action(db=db,action_id=action_id)
    db_action.status= status
    db_action.finished= finished

    db.flush()
    db.commit()
    db.refresh(db_action)
    return db_action

def delete_action(db:Session, action_id):
    db_action = get_action(action_id= action_id,db=db)
    db.delete(db_action)
    db.commit()
    
    db_sub_action= db.query(models.DrawerAction).filter(models.DrawerAction.id== action_id)
    if db_sub_action is None:
        db_sub_action=db.query(models.NavigationAction).filter(models.NavigationAction.id== action_id)
    elif db_sub_action is None:
        db_sub_action=db.query(models.NewUserAction).filter(models.NewUserAction.id== action_id)
    
    db.delete(db_sub_action)
    db.commit()
    
def get_next_task(db:Session, robot_name):
    new_task=tasks_queue(db)[0]
    return new_task

def get_full_task(task_id, db):
    
    db_task = get_task( db=db, task_id=task_id)
    if db_task is None:
        return "task"
        raise HTTPException(status_code=404, detail="task not found")
    task_id=db_task.id
    db_actions= get_actions( db_task.id,db)
    if db_actions is None: 
        return "action"
    action_list=[]
    for db_action in db_actions:
        step = db_action.step
        action_type = db_action.type
        if(db_action.type==schemas.ActionType.DRAWER):
           
           db_drawer_action= get_drawer_action(db_action.id, db)
           if db_drawer_action is None: 
            return "action"
           drawer_id=db_drawer_action.drawer_id
        #    locked_for= [db_drawer_action.target_user_id]
           module_id=db_drawer_action.module_id
           drawer_action= schemas.Drawer( drawer_id=drawer_id, module_id=module_id, locked_for=[] )
           action= schemas.Action(step=step, type=action_type,action=drawer_action)
           
        elif db_action.type== schemas.ActionType.NAVIGATION:
            db_nav_action= get_navigation_action(db_action.id, db)
            if db_drawer_action is None: 
                return "action"
            
            posex=db_nav_action.x_pose
            posey= db_nav_action.y_pose
            posez=0
            pose=schemas.Pose(x=posex,y=posey,z=posez)
            yaw=db_nav_action.yaw_pose
            nav_action=schemas.Navigation(pose=pose, yaw=yaw)
            action.action=schemas.Action(step=step, type=action_type, action=nav_action)

        elif db_action.type== schemas.ActionType.NEW_USER:
            db_user_action= get_user_action(db_action.id, db)
            if db_user_action is None: 
                return "action"
            user_id=db_user_action.user_id
            user_action= schemas.NewUser(user_id)
            action= schemas.Action(step=step,type=action_type, action= user_action)
       
        action_list.append(action)
    return schemas.Task(task_id=str(task_id), actions=action_list)
#robot
def get_robots(db:Session, skip: int = 0, limit: int = 100)->[models.Robot]:
    db_robots= db.query(models.Robot).offset(skip).limit(limit).all()
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
    return db_robot
        
#Modules 
def get_modules(db: Session, robot_name: str)->[models.Module]:
    return db.query(models.Module).filter(robot_name== models.Module.robot_name ==robot_name).all()

def get_module(db: Session, module_id: int)-> models.Module:
    return db.query(models.Module).filter(models.Module.module_id == module_id).all()
def get_module(db: Session, module_id: int, drawer_id:int)-> models.Module:
    return db.query(models.Module).filter(models.Module.module_id == module_id and models.Module.drawer_id== drawer_id).first()

def get_drawer(db: Session,robot_name:str, module_id: int, drawer_id: int)-> models.Module:
    drawer= db.query(models.Module).filter(models.Module.robot_name ==robot_name, models.Module.module_id == module_id, models.Module.drawer_id== drawer_id).first()
    return drawer

def set_module(db:Session, module: schemas.Module)-> models.Module:
    db_module= get_drawer(db=db, robot_name= module.robot_name, module_id= module.module_id, drawer_id = module.drawer_id)
    if(db_module is None):
        db_module = models.Module(id = module.module_id,
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
    db_module= get_drawer(db=db, robot_name= module.robot_name, module_id= module.module_id, drawer_id = module.drawer_id)
    if(db_module is None):
        return 
    else:
        db_module.status= module.status
        db_module.label= module.label
        db_module.robot_name= module.robot_name
    db.commit()
    db.refresh(db_module)
    return db_module



