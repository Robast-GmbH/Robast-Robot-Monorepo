from typing import List
from fastapi.middleware.cors import CORSMiddleware
import json
import yaml
import uvicorn
from fastapi import Depends, FastAPI, HTTPException
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

@app.post("/users/NFC")
def add_nfc_code_to_user(user: schemas.UserBase, db: Session = Depends(get_db)):
   #todo
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
    db_tasks = crud.get_tasks(db)
    return db_tasks

@app.get("/tasks/{task_id}", response_model=schemas.Task)
def read_task(task_id: int, db: Session = Depends(get_db)):
    db_task = crud.get_task(db, task_id=task_id)
    if db_task is None:
        raise HTTPException(status_code=404, detail="task not found")
    return db_task

@app.post("/tasks/", response_model=schemas.TaskDelivery)
def create_task(task: schemas.TaskDelivery, db: Session = Depends(get_db)):
    return crud.create_task(db=db, task=task)

@app.put("/tasks/{task_id}", response_model=schemas.Task)
def update_task( task_id:int, task: schemas.TaskUpdate, db: Session = Depends(get_db)):
    return crud.update_task(db=db, task_id = task_id, content=task)
    

@app.delete("/tasks/{task_id}")
def abort_task(task_id: int, db: Session = Depends(get_db)):
    if(not crud.delete_task(db = db, task_id = task_id)):
        raise HTTPException(status_code=404, detail="User not found")
       
#robot
@app.get("/robots", response_model =List[schemas.Robot])
def get_robots_status(db: Session = Depends(get_db)):
    db_robots = crud.get_robots(db=db)
    if db_robots is None:
        return []
    return db_robots

@app.get("/robots/{robot_name}", response_model = schemas.Robot)
def get_robot_status(robot_name: str, db: Session = Depends(get_db)):
    db_robot = crud.get_robot(db=db, robot_name=robot_name)
    return db_robot

@app.put("/robots/halt")
def pause_robot(robot_name: str, halt: bool , db: Session = Depends(get_db)):
    #todo
    return

@app.put("/robots/rosbag")
def set_rosbag(robot_name: str, halt: bool , db: Session = Depends(get_db)):
    #todo
    return

@app.put("/robots/status")
def set_robot(robot: schemas.Robot, db: Session = Depends(get_db)):
    return crud.set_robot(db=db, robot=robot)

@app.put("/robots/{robot_name}/move")
def move_robot( robot_name: str, x: float, y: float, yaw: float, db: Session = Depends(get_db)):
    db_robot= crud.get_robot(db=db, robot_name=robot_name)
    if db_robot is None:
        raise HTTPException(status_code=404, detail="robot not found")
    robot= templates.json_robot()
    robot["name"]= db_robot.robot_name
    robot["fleet_name"]= db_robot.fleet_name

    waypoint= templates.json_waypoint()
    waypoint["pose.x"]=x
    waypoint["pose.y"]=y
    waypoint["pose.z"]=0
    waypoint["orientation"]=yaw
    
    message={"robot":robot, "waypoint":waypoint}
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session()
    answer  =sender.post(url= config["fleetmangement_address"]+"/move", json= json.dumps(message), headers= headers, verify=False)
    return answer.status_code


@app.get("/robots/{robot_name}/log", response_model = List[str])
def get_log(robot_name: str, db: Session = Depends(get_db)):
   #todo
    return None

#Module
@app.get("/robots/{robot_name}/modules/", response_model = List[schemas.ModuleBase])
def get_modules(robot_name :str, db: Session = Depends(get_db)):
    return crud.get_drawers(robot_name)

# get_module_info
@app.get("/robots/{robot_name}/modules/{module_id}", response_model = schemas.ModuleBase)
def get_drawer( robot_name: str, module_id:int, db: Session = Depends(get_db)):
    return crud.get_drawer(robot_name, module_id)

#set_drawer
@app.post("/robots/modules/",)
def update_drawer(module: schemas.Module, db: Session = Depends(get_db)):
    return crud.set_module(db=db, module=module)
 
# open_drawer
@app.post("/robots/{robot_name}/modules/open")
def open_drawer( robot_name: str, module: schemas.ModuleBase ,db: Session = Depends(get_db)):
    db_module= crud.get_drawer(db=db, module_id=module.id, drawer_id= module.drawer_id,robot_name=robot_name )
    db_robot= crud.get_robot(db=db, robot_name=robot_name)
    if db_robot is None:
        raise HTTPException(status_code=404, detail="robot not found")
    
    if db_module is None:
        raise HTTPException(status_code=404, detail="module not found")

    robot= templates.json_robot()
    robot["name"]= db_robot.robot_name
    robot["fleet_name"]= db_robot.fleet_name
    drawer= templates.json_drawer()
    drawer["id"]=db_module.drawer_id
    drawer["fleet_name"]= db_robot.fleet_name
    message={"robot":robot, "drawer":drawer}
    headers =  {"Content-Type":"application/json"}
    sender = requests.Session()
    answer  =sender.post(url= config["fleetmangement_address"]+"/drawer/open", json= json.dumps(message), headers= headers, verify=False)
    return answer.status_code

# close_drawer
@app.post("/robots/{robot_name}/modules/close")
def close_drawer( robot_name: str, module: schemas.ModuleBase , db: Session = Depends(get_db)):
    db_module= crud.get_drawer(module_id=module.id,drawer_id= module.drawer_id )
    if db_module is not None:
        robot= templates.json_robot()
        robot["name"]=robot_name
        robot["fleet_name"]=""
        drawer= templates.json_drawer()
        drawer["id"]=db_module
        drawer["fleet_name"]= drawer.drawer_id
        message={"robot":robot, "drawer":drawer}
        headers =  {"Content-Type":"application/json"}
        sender = requests.Session()
        answer  =sender.post(url= config["fleetmangement_address"]+"/drawer/close", json= json.dumps(message),headers= headers)
        return answer.status_code

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port= config['restapi_port'])
   