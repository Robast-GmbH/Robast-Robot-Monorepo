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

config = yaml.safe_load(open("./config.yml"))


origins = [
    "http://localhost:"+str(config['restapi_port']),
    "http://127.0.0.1:"+str(config['restapi_port']),
    "http://172.18.0.2:"+str(config['restapi_port'])
    
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


#User management
@app.post("/users/", response_model=schemas.User)
def create_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = crud.get_user_by_email(db, email=user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    return crud.create_user(db=db, user=user)

@app.post("/users/nfc")
def add_nfc_code_to_user(user_id:int, robot_name:str, db: Session = Depends(get_db)):
    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, user_id)
    step=1
    db_action= crud.create_new_user_nfc(db, step, task_id,user_id )
    action= templates.create_new_user(step, user_id= user_id)
    task =templates.create_task(db,robot_name,task_id,[action])
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["fleetmangement_address"]+"/task", data= json.dumps(task), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return  

@app.post("/users/{user_id}/nfc")
def update_nfc_code(user_id:str, nfc_code:str, db: Session = Depends(get_db)):
    crud.create_nfc_code(user_id=user_id,nfc_code=nfc_code)
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
@app.get("/tasks/", response_model=List[schemas.Task])
def get_task_queue( db: Session = Depends(get_db)):
    db_tasks = crud.get_tasks_queue(db)
    
    if len(db_tasks) ==0:
            
            return[]
    return db_tasks

@app.get("/tasks/{task_id}", response_model=schemas.ActiveTask)
def read_task(task_id: int, db: Session = Depends(get_db)):
    db_task = crud.get_task(db, task_id=task_id)
    if db_task is None:
        raise HTTPException(status_code=404, detail="task not found")
    return db_task

@app.post("/tasks/")
def create_task(task: schemas.Task, robot:schemas.Robot, user_id:int, db: Session = Depends(get_db)):
    task_id= crud.create_task(db=db,robot_name= robot.robot_name, owner_id= user_id)
    for action in task.actions:
        if( action.type== schemas.ActionType.MOVE):
            crud.create_move_action(db, action.step, task_id, action.action.pose.x, action.action.pose.y, action.action.orientation)
        elif(action.type== schemas.ActionType.OPEN_DRAWER):
            crud.create_drawer_action(db, action.step,task_id, action.action.module_id, action.action.drawer_id)
        elif(action.type== schemas.ActionType.NEW_USER):
            crud.create_new_user_nfc(db, action.step, task_id, action.action.user_id)
    return 

@app.put("/tasks/{task_id}", response_model=schemas.Task)
def update_task( task_id:int, task: schemas.BaseTask, db: Session = Depends(get_db)):#check
    return crud.update_task(db=db, task_changes=task)#task

#robot
@app.get("/robots", response_model =List[schemas.RobotStatus])
def get_robots_status(db: Session = Depends(get_db)):
    db_robots = crud.get_robots(db=db)
    return db_robots

@app.get("/robots/{robot_name}", response_model = schemas.RobotStatus)
def get_robot_status(robot_name: str, db: Session = Depends(get_db)):
    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    return db_robot

@app.put("/robots/pause_resume")
def pause_robot(robot_name: str, fleet_name:str, pause: bool , db: Session = Depends(get_db)):
    headers = {"Content-Type":"application/json"}
    sender = requests.Session() 
    message= {"robot_name":robot_name, "fleet_name":fleet_name}
    
    if(pause):
        answer  =sender.post(url= config["fleetmangement_address"]+"/settings/move/pause",json= json.dumps(message), headers= headers, verify=False)
    else:
        answer  =sender.post(url= config["fleetmangement_address"]+"/settings/move/resume",json= json.dumps(message), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 


@app.put("/robots/status")
def set_robot(robot: schemas.Robot, db: Session = Depends(get_db)):
    return crud.set_robot(db=db, robot=robot)

@app.put("/robots/{robot_name}/move")
def move_robot( robot_name: str, x: float, y: float, yaw: float, owner_id:int, db: Session = Depends(get_db)):
    db_robot= crud.get_robot(db=db, robot_name=robot_name)
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, owner_id)
    step=1
    db_action= crud.create_move_action(db, step, task_id,x,y,yaw)
    action= templates.create_move_action( step, x, y, yaw)
    task =templates.create_task(db,robot_name,task_id,[action])
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["fleetmangement_address"]+"/task", data= json.dumps(task), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

#Module
@app.get("/robots/{robot_name}/modules/", response_model = List[schemas.Module])
def get_modules(robot_name :str, db: Session = Depends(get_db)):
    return crud.get_modules(db=db, robot_name=robot_name)

# get_module_info
@app.get("/robots/{robot_name}/modules/{module_id}", response_model = schemas.Module)
def get_drawer( robot_name: str, module_id:int, db: Session = Depends(get_db)):
    return crud.get_drawer(db=db, robot_name=robot_name, module_id= module_id)

#set_drawer
@app.post("/robots/modules/", response_model= schemas.Module)
def update_drawer(module: schemas.Module, db: Session = Depends(get_db)):
    return crud.set_module(db=db, module=module)

@app.post("/robots/modules/status",)
def update_drawer_status(module: schemas.UpdateModule, db: Session = Depends(get_db)):
    return crud.set_module_status(db=db, module=module)
 
# finish_using_drawers
@app.post("/robots/{robot_name}/modules/completed")
def drawer_actions_done( robot_name: str, fleet_name:str, db: Session = Depends(get_db)):

    message= {"robot_name":robot_name, "fleet_name":fleet_name}
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["fleetmangement_address"]+"/settings/drawer/completed", data= json.dumps(message), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 

# open_drawer
@app.post("/robots/{robot_name}/modules/open")
def open_drawer( robot_name: str, module: schemas.BaseDrawer, restricted_for_user: list[int], owner:Annotated[int,Body()], db: Session = Depends(get_db)):
    
    nfc_codes=[]
    if(len(restricted_for_user)>0):
        for id in restricted_for_user:
            nfc_code= crud.get_nfc_code(db=db, user_id=id)
            nfc_codes.append(str(id)+":"+nfc_code)

    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    db_module= crud.get_module(db=db,module_id=module.module_id,drawer_id=module.drawer_id)
    if db_module is None:
        raise HTTPException(status_code=404, detail="module not found" )
    
    task_id= crud.create_task( db, db_robot.robot_name, db_robot.fleet_name, owner)
    step=1
    db_action= crud.create_drawer_action(db, step, task_id, module.module_id, module.drawer_id)
    action= templates.create_drawer_action(step,module.drawer_id, module.module_id, db_module.is_edrawer, nfc_codes)
    task =templates.create_task(db,robot_name,task_id,[action])
    print(json.dumps(task),)
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session() 
    answer  =sender.post(url= config["fleetmangement_address"]+"/task", data= json.dumps(task), headers= headers, verify=False)

    if answer.status_code!= 200:
        raise HTTPException(status_code=answer.status_code, detail= answer.reason)
    return 


# close_drawer
@app.post("/robots/{robot_name}/modules/close")
def close_drawer( robot_name: str, module: schemas.BaseDrawer , db: Session = Depends(get_db)):
        message=templates.get_close_drawer_interaction_json(db, robot_name, module.drawer_id, module.module_id, module.is_edrawer)
    
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session()
        answer  =sender.post(url= config["fleetmangement_address"]+"/settings/drawer/close", data= json.dumps(message),headers= headers)

        if answer.status_code!= 200:
            raise HTTPException(status_code=answer.status_code, detail= answer.reason)
        return 

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port= config['restapi_port'])
   

