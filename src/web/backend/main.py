from typing import List
from fastapi.middleware.cors import CORSMiddleware
import json
import yaml
import uvicorn
from fastapi import Depends, FastAPI, Body, HTTPException
from typing_extensions import Annotated
from sqlalchemy.orm import Session
import requests
import helper as templates
import crud
import models
import schemas
from database import SessionLocal, engine

models.Base.metadata.create_all(bind=engine)

app = FastAPI()
loop_task_id=None
config = yaml.safe_load(open("./src/config.yml"))


origins = [
    "http://localhost:"+str(config['RESTAPI_PORT']),
    "http://127.0.0.1:"+str(config['RESTAPI_PORT']),
    "http://172.18.0.2:"+str(config['RESTAPI_PORT'])
    
]

app.add_middleware(
    CORSMiddleware,
    allow_origins="*",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Dependency
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@app.get("/")
async def read_main(db: Session = Depends(get_db)):
    crud.init(db)
    return {"msg": "Hi Robast Fans. LETS GET STARTED!!!!"}

#reset DB to initial_state
@app.get("/reset")
def reset_db(db:Session=Depends(get_db)):
    models.Base.metadata.drop_all(bind=engine)
    models.Base.metadata.create_all(bind=engine)
    crud.init(db)
    return{ "msg":"Database reset successful."}

#User management
@app.post("/users/", response_model=schemas.User)
def create_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = crud.get_user_by_email(db, email=user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    return crud.create_user(db=db, user=user)

@app.post("/users/{user_id}/nfc")
def update_nfc_code(user_id:str, robot_name="", db: Session = Depends(get_db)):
    db_robot= crud.get_robot(db=db, robot_name=robot_name)
    db_user=crud.get_user(db,user_id)
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, user_id)
    step=1
    db_action= crud.create_new_user_nfc(db, step, task_id, db_user.id)
    action= templates.create_new_user( step=step, user_id= db_user.id)
    task =templates.create_task(db,robot_name,task_id,[action])
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    print(json.dumps(task))
    answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/task", data= json.dumps(task), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

@app.post("/users/{user_id}/admin/nfc")
def update_nfc_code_in_db(user_id:str, nfc_code:str="", db: Session = Depends(get_db)):
    crud.create_nfc_code(db= db, user_id=user_id,nfc_code=nfc_code)
    return 

@app.post("/users/login")#, response_model=schemas.User
def login( userCredentials: schemas.UserLogin, db: Session = Depends(get_db)):
    user = crud.get_users_login(db, userCredentials )
    if user is None:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@app.get("/users/", response_model=List[schemas.User])
def read_users(skip: int = 0, limit: int = 100, db: Session = Depends(get_db)):
    users = crud.get_users(db, skip=skip, limit=limit)
    return users

@app.get("/users/{user_id}", response_model=schemas.User)
def read_user(user_id: int, db: Session = Depends(get_db)):
    db_user = crud.get_user(db, user_id=user_id)
    if db_user is None:
        raise HTTPException(status_code=404, detail="User not found")
    return db_user

@app.delete("/users/{user_id}")
def delete_user(user_id: int, db: Session = Depends(get_db)):
    if(not crud.delete_user(db = db, user_id = user_id)):
        raise HTTPException(status_code=404, detail="User not found")

#task
@app.get("/tasks/",response_model=list[schemas.Task])
def get_task_queue( db: Session = Depends(get_db)):
    db_tasks = crud.tasks_queue(db)
    db_complete_tasks=[]
    for task in db_tasks:
        full_task=crud.get_full_task(task_id=task.id,db=db)
        if not(full_task=="task" or full_task=="action"):
                db_complete_tasks.append(full_task)
    return db_complete_tasks

@app.get("/tasks/{task_id}", response_model=schemas.Task)
def read_task(task_id: int, db: Session = Depends(get_db)):
    db_task= crud.get_full_task(task_id, db)
    if db_task=="task":
        raise HTTPException(status_code=404, detail="Task not found")
    elif db_task=="action":
        raise HTTPException(status_code=404, detail="Action not found")
    return db_task

@app.post("/tasks/", response_model=int)
def create_task(task: schemas.Task, robot:schemas.Robot, user_id:int, db: Session= Depends(get_db)):
    task_id= crud.create_task(db=db,robot_name= robot.robot_name, fleet_name=robot.fleet_name, owner_id= user_id)
    for action in task.actions:
        if( action.type== schemas.ActionType.NAVIGATION):
            crud.create_navigation_action(db, action.step, task_id, action.action.pose.x, action.action.pose.y, action.action.yaw)
        elif(action.type== schemas.ActionType.DRAWER):
            crud.create_drawer_action(db, action.step,task_id, action.action.module_id, action.action.drawer_id,action.action.owner_id, action.action.locked_for)
        elif(action.type== schemas.ActionType.NEW_USER):
           crud.create_new_user_nfc(db, action.step, task_id, action.action.user_id)
    return task_id

@app.put("/tasks/{task_id}", response_model=schemas.Task)
def update_task( task_id:int, task: schemas.BaseTask, db: Session = Depends(get_db)):
    return crud.update_task(db, task.task_id, task.robot.robot_name, task.robot.fleet_name)

@app.put("/tasks/{task_id}/step/{step}",response_model=schemas.UpdateAction)
def update_action(task_id:str, step:str, status:Annotated[str,Body()],finished:Annotated[bool,Body()], db: Session = Depends(get_db)):
    action_id= crud.get_action_id(db=db,task_id=task_id,step=step)
    return crud.update_action(db=db, action_id=action_id, status=status, finished=finished)

@app.get("/robots/{robot_name}/next_task")#, response_model=schemas.Task)
def get_next_task(robot_name: str, db: Session = Depends(get_db)):
    next_task=None
    
    db_tasks_queue = crud.tasks_queue(db)
    for task in db_tasks_queue:
        next_task=crud.get_full_task(task_id=task.id,db=db)
        if(not(next_task == "task" or next_task== "action")):
            break
        next_task=None

    if(next_task is None):
        if(loop_task_id is None):
            return
        next_task=crud.get_next_patrol_action(task_id= loop_task_id, db=db)
    
    db_robot=crud.get_robot(db=db,robot_name=robot_name)
    crud.update_task(db=db, task_id=next_task.task_id, robot_name= db_robot.robot_name, fleet_name=db_robot.fleet_name)

    actions= templates.create_action_list(db=db, robot_name=robot_name, actions=next_task.actions)
    task =templates.create_task(db=db, robot_name=robot_name, task_id= next_task.task_id,  action=actions )

    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/task", data= json.dumps(task), headers= headers, verify=False)
    
    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return task

#robot
@app.get("/robots", response_model =List[schemas.RobotStatus])
def get_robots_status(db: Session = Depends(get_db)):
    db_robots = crud.get_robots(db=db)
    return db_robots

@app.get("/robots/{robot_name}", response_model = schemas.RobotStatus)
def get_robot_status(robot_name: str, db: Session = Depends(get_db)):
    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    return db_robot

@app.delete("/robots")
def delete_all_robots(db: Session = Depends(get_db)):
    crud.delete_robots(db=db)
    return 


@app.put("/robots/pause_resume")
def pause_robot(robot_name: str, fleet_name:str, pause: bool , db: Session = Depends(get_db)):
    headers = {"Content-Type":"application/json"}
    sender = requests.Session() 
    message= {"robot_name":robot_name, "fleet_name":fleet_name}
  
    if(pause):
        answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/settings/navigation/pause",json= json.dumps(message), headers= headers, verify=False)
    else:
        answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/settings/navigation/resume",json= json.dumps(message), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

@app.put("/robots/loop/start")
def set_loop( task: schemas.Task, user_id:int, force_start:bool, db: Session = Depends(get_db)):
    if( not all(action.type== schemas.ActionType.NAVIGATION for action in task.actions)):
          raise HTTPException(status_code=404, detail="patrol task can only consists of navigate actions" )
    loop_task_id = create_task(task, task.robot, user_id, db)
    
    if force_start:
        task=crud.get_full_task(task_id=loop_task_id, db=db)
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session() 
        answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/task", data= json.dumps(templates.json_task(task)), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

@app.put("/robots/loop/stop")
def reset_loop():
    loop_task_id=None

@app.post("/robots/status", response_model= schemas.RobotStatus)
def set_robot(robot: schemas.RobotStatus, db: Session = Depends(get_db)):
    return crud.set_robot(db=db, override=True, robot=robot)

@app.put("/robots/{robot_name}/navigate")
def navigate_robot( robot_name: str, target:schemas.Navigation, owner_id:Annotated[int,Body()], db: Session = Depends(get_db)):
    db_robot= crud.get_robot(db=db, robot_name=robot_name)
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, owner_id)
    step=1
    db_action= crud.create_navigation_action(db, step, task_id,target.pose.x,target.pose.y,target.yaw)
    action= templates.create_navigation_action( step, target.pose.x, target.pose.y, target.yaw)
    task =templates.create_task(db,robot_name,task_id,[action])
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/task", data= json.dumps(task), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

#Module
@app.get("/robots/{robot_name}/modules/", response_model = List[schemas.Module])
def get_modules(robot_name :str, db: Session = Depends(get_db)):
    db_modules=crud.get_modules(db=db, robot_name=robot_name)
    for module in db_modules:
        if(module.status=="Unlocked"):
            module.status="Opened"  
    return db_modules

# get_module_info
@app.get("/robots/{robot_name}/modules/{module_id}", response_model = schemas.Module)
def get_drawer( robot_name: str, module_id:int, db: Session = Depends(get_db)):
    return crud.get_drawer(db=db, robot_name=robot_name, module_id= module_id, drawer_id=0)

@app.get("/robots/{robot_name}/modules/status/reset")
def clear_module_status( robot_name:str, db: Session = Depends(get_db)):
    crud.set_drawers_to_close(db=db, robot_name=robot_name)
    return 

#set_drawer
@app.post("/robots/modules/", response_model= schemas.Module)
def update_drawer(module: schemas.Module, db: Session = Depends(get_db)):
    return crud.set_module(db=db, module=module)

@app.post("/robots/modules/status")
def update_drawer_status(module: schemas.UpdateModule, db: Session = Depends(get_db)):
    crud.remove_unlocked_state(db)
    db_module= crud.get_module(db=db, module_id=module.module_id,drawer_id= module.drawer_id)
    if(module.label is None):
        module.label= db_module.label
    if(module.status  is None):
        module.status= db_module.status
    if(module.robot_name  is None):
        module.robot_name=db_module.robot_name

    return crud.set_module_status(db=db, module=module)
 
# finish_using_drawers
@app.post("/robots/{robot_name}/modules/completed")
def drawer_actions_done( robot_name: str, fleet_name:str, db: Session = Depends(get_db)):

    message= {"robot_name":robot_name, "fleet_name":fleet_name}
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/settings/drawer/completed", data= json.dumps(message), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

# open_drawer
@app.post("/robots/{robot_name}/modules/open")
def open_drawer( robot_name: str, module: schemas.BaseDrawer, restricted_for_user: list[int], owner:Annotated[int,Body()], db: Session = Depends(get_db)):
    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    db_module= crud.get_module(db=db,module_id=module.module_id,drawer_id=module.drawer_id)
    if db_module is None:
        raise HTTPException(status_code=404, detail="module not found" )
    
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, owner)
    step=1
    db_action= crud.create_drawer_action(db, step, task_id, module.module_id, module.drawer_id, locked_for=restricted_for_user)
    action= templates.create_drawer_action(step,module.drawer_id, module.module_id, db_module.type == models.DrawerSlideTypes.Electrical, restricted_for_user,False)
    if(action== "robot"):
        raise HTTPException(status_code=404, detail="robot not found" )
    task =templates.create_task(db,robot_name,task_id,[action])
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/task", data= json.dumps(task), headers= headers, verify=False)
    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= json.dumps(task))
    return 


# close_drawer
@app.post("/robots/{robot_name}/modules/close")
def close_drawer( robot_name: str, module: schemas.BaseDrawer , db: Session = Depends(get_db)):
        db_module= crud.get_module(db, module.module_id, module.drawer_id)
        if db_module is None:
            raise HTTPException(status_code=404, detail="module not found" )
        message=templates.get_close_drawer_interaction_json(db, robot_name, module.drawer_id, module.module_id, db_module.type== models.DrawerSlideTypes.Electrical)
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session()
        answer  =sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/settings/drawer/close", data= json.dumps(message),headers= headers)

        if answer.status_code!= 200:
            raise HTTPException(status_code=answer.status_code, detail= answer.reason)
        return 

#reset normal drawer
@app.post("/robots/{robot_name}/modules/reset")
def reset_drawer( robot_name: str, db: Session = Depends(get_db)):
    headers =  {"Content-Type":"application/json"}
    message=templates.find_robot_json(db, robot_name)
    if message== "robot":
        raise HTTPException(status_code=404, detail="robot not found" )
    sender = requests.Session()
    answer = sender.post(url= config["FLEETMANAGEMENT_ADDRESS"]+"/settings/drawer/reset", data= json.dumps(message), headers= headers)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    db_modules= crud.get_modules(db=db,robot_name=robot_name)
    for module in db_modules:
        mod=schemas.UpdateModule(module_id=module.module_id, drawer_id=module.drawer_id, robot_name=robot_name ,status="Closed")
        crud.set_module_status(db=db,module=mod)
    return 

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port= config['RESTAPI_PORT'])
   

