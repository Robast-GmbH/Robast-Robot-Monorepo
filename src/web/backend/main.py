from typing import List
from urllib import response
from fastapi.encoders import jsonable_encoder
from sqlalchemy.sql.functions import user
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

import uvicorn
from fastapi import Depends, FastAPI, HTTPException
from sqlalchemy.orm import Session


import crud
import models
import schemas
from database import SessionLocal, engine


models.Base.metadata.create_all(bind=engine)


app = FastAPI()



origins = [
    "http://localhost:3001",
    "http://127.0.0.1:3001",
    "http://172.18.0.2:3001"
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
def get_task_queue(user_id: int, db: Session = Depends(get_db)):
    db_tasks = crud.get_tasks(db)
    return db_tasks

@app.post("/tasks/", response_model=schemas.TaskCreate)
def create_task(task: schemas.TaskCreate, db: Session = Depends(get_db)):
    return crud.create_task(db=db, task=task)

@app.put("/task/{task_id}", response_model=schemas.Task)
def update_task( task_id:int, task: schemas.TaskUpdate, db: Session = Depends(get_db)):
    return crud.update_task(db=db, task_id = task_id, content=task)
    
#abort_task
@app.delete("/tasks/{task_id}")
def delete_task(task_id: int, db: Session = Depends(get_db)):
    if(not crud.delete_task(db = db, task_id = task_id)):
        raise HTTPException(status_code=404, detail="User not found")
       
#robot

#pause/resume
@app.put("/robots/{robot_name}/halt")
def pause_robot(robot_name: str, halt: bool , db: Session = Depends(get_db)):
    #todo
    return

#rosbag->start/stop
@app.put("/robots/{robot_name}/rosbag")
def set_rosbag(robot_name: str, halt: bool , db: Session = Depends(get_db)):
    #todo
    return

#get status
@app.get("/robots/{robot_name}/status",response_model=schemas.Robot)
def get_robot_status(robot_name: str, db: Session = Depends(get_db)):
    db_tasks = crud.get_tasks(db)
    return db_tasks

#set_status
@app.put("/robots/{robot_name}/status")
def set_robot(robot_name: str, db: Session = Depends(get_db)):
    #todo
    return

    #get_position
    #set_position
    #move_to_point
    #debug_log

#drawer
    # get_installed_drawers
    # get_drawer_info
    # open_drawer
    #close_drawer
    # status_status
    # get_unlocked_drawers

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=3002)
    