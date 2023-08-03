from typing import List
from urllib import response
from fastapi.encoders import jsonable_encoder
from sqlalchemy.sql.functions import user
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

import uvicorn
from fastapi import Depends, FastAPI, HTTPException
from sqlalchemy.orm import Session

import coords_setup
import crud
import models
import schemas
import helper
from coords_setup import readMapSetup
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


@app.post("/map/", response_model=bool)
def get_or_create_map(db: Session = Depends(get_db), name="map_setup"):
    rooms = crud.get_map_by_name(db, name=name)
    if not rooms:
        rooms_yaml = readMapSetup(name+".yaml")
        if rooms_yaml:
            rooms = crud.create_roomMap(db, schemas.RoomsCreate(map_name=name, map_yaml=rooms_yaml))
        else:
            raise HTTPException(status_code=400, detail="No map with the given name")
    return True


@app.post("/users/", response_model=schemas.User)
def create_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = crud.get_user_by_email(db, email=user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    return crud.create_user(db=db, user=user)

@app.post("/users/login")#, response_model=schemas.User
def login( userCredentials: schemas.UserLogin, db: Session = Depends(get_db)):
    users = crud.get_users_login(db, userCredentials )
    if user is None:
        raise HTTPException(status_code=404, detail="User not found")
    return users

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


@app.delete("/users/{user_id}", response_model=bool)
def delete_user(user_id: int, db: Session = Depends(get_db)):
    if(not crud.delete_user(db = db, user_id = user_id)):
        raise HTTPException(status_code=404, detail="User not found")
    

@app.post("/users/{user_id}/order/", response_model=schemas.Order)
def create_order_for_user(
    user_id: int, order: schemas.OrderCreate, db: Session = Depends(get_db)
):
    order = crud.create_user_order(db=db, order=order, user_id=user_id)
    if(order == None):
        raise HTTPException(status_code=404, detail="Order could not be added to User {user_id}")
    return order


@app.get("/orders/", response_model=List[schemas.Order])
def orders(db: Session = Depends(get_db)):
    return crud.get_orders(db)


@app.get("/orders/{order_id}", response_model=schemas.Order)
def get_order(order_id: int, db: Session = Depends(get_db)):
    return crud.get_order(db, order_id= order_id)


@app.put("/orders/{order_id}", response_model=schemas.Order)
def update_order(order_id: int, order: schemas.OrderCreate, db: Session = Depends(get_db)):
    return crud.update_order(db=db, order_id= order_id, order=order)
    

@app.delete("/orders/{order_id}")
def delete_order(order_id: int, db: Session = Depends(get_db)):    
    if(not crud.delete_order(db=db, order_id= order_id)):
        raise HTTPException(status_code=404, detail="Order not found")
    return

@app.get("/drawers/", response_model=List[schemas.Drawer])
def drawer(db: Session = Depends(get_db)):
    return crud.get_drawers(db)

@app.get("flowa/drawers/", response_class=JSONResponse)
def drawer(db: Session = Depends(get_db)):
    return {"Drawers": crud.get_drawers(db)}

@app.get("/drawers/{position}", response_model=schemas.Drawer)
def get_drawer( position:int, db: Session = Depends(get_db)):
    return crud.get_drawer(db, position = position)


@app.post("/drawers/", response_model=schemas.Drawer)
def create_drawer( drawer: schemas.DrawerCreate, db: Session = Depends(get_db)
):
    if(drawer.drawer_controller_id== 0):
        raise HTTPException(status_code=400, detail="controller ID not allowed.")
    
    drawer = crud.create_drawer(db=db, drawer=drawer)
    if(drawer == None):
        raise HTTPException(status_code=404, detail="Drawer could not be Created as it already exists.")
    return drawer


@app.get("/drawer/open/", response_model=int )
def get_open_drawer():
     return helper.open_drawer

@app.put("/drawer/{drawer_controller_id}/open/", response_model=schemas.Drawer)
def set_open_drawer(drawer_controller_id:int, db: Session = Depends(get_db)):
    helper.open_drawer =drawer_controller_id
    return crud.get_drawer(db=db, drawer_controller_id=drawer_controller_id)

@app.put("/flowa/drawer/open/", response_model=schemas.Drawer)
def set_open_drawer(drawer_controller_id:int, db: Session = Depends(get_db)):
    helper.open_drawer =drawer_controller_id
    return crud.get_drawer(db=db, drawer_controller_id=drawer_controller_id)
     

@app.delete("/drawer/open/")
def delete_open_drawer():
    helper.open_drawer=-1
    return 

@app.get("/drawer/empty/", response_model=list[schemas.Drawer])
def get_open_drawer(db: Session = Depends(get_db)):
     return crud.get_empty_drawer(db=db)

@app.put("/drawer/{id}/empty/")
def set_drawer_to_empty(id:int, empty:bool, db: Session = Depends(get_db)):
     crud.change_drawer_empty_state(db=db,drawer_controller_id= id, empty= empty)
     return 


@app.put("/drawers/{drawer_controller_id}", response_model=schemas.Drawer)
def update_drawer( drawer_controller_id:int, drawer: schemas.DrawerCreate, db: Session = Depends(get_db)):
    return crud.update_drawer(db=db, drawer_controller_id = drawer_controller_id, content=drawer.content)
    

@app.delete("/drawers/{drawer_controller_id}")
def delete_drawer( drawer_controller_id:int, db: Session = Depends(get_db)):    
    if(not crud.delete_drawer(db=db, posdrawer_controller_idtion =drawer_controller_id)):
        raise HTTPException(status_code=404, detail="Drawer not found")
    return


@app.get("/map_positions/", response_model=List[schemas.MapPosition])
def map_position(db: Session = Depends(get_db)):
    return crud.get_map_positions(db)


@app.get("/map_positions/{id}", response_model=schemas.MapPosition)
def get_map_position(id: int, db: Session = Depends(get_db)):
    return crud.get_map_position(db, id = id)


@app.post("/map_positions/", response_model=schemas.MapPosition)
def create_map_position( map_position: schemas.MapPositionCreate, db: Session = Depends(get_db)
):
    map_position = crud.create_map_position(db=db, map_position=map_position)
    if(map_position == None):
        raise HTTPException(status_code=404, detail="Map position could not be created as it already exists.")
    return map_position


@app.delete("/map_positions/{id}")
def delete_map_position(id: int, db: Session = Depends(get_db)):    
    if(not crud.delete_map_position(db=db, id =id)):
        raise HTTPException(status_code=404, detail="Map position not found")
    return


@app.get("/robot/status/", response_model=int )
def get_robot_status():
     return helper.robot_status

@app.put("/robot/status/", response_model= int)
def update_robot_status( status:int ):
    if(status<4):
        helper.robot_status =status
    else:
        raise HTTPException(status_code=404, detail="Status in valid")
    return helper.robot_status


@app.get("/robot/goal/", response_model= schemas.MapPosition )
def get_next_goal(db: Session = Depends(get_db)):
     if(helper.next_point==-1):
        raise HTTPException(status_code=404, detail="No goal found.")
     return crud.get_map_position(db, helper.next_point)

@app.put("/robot/goal/" )
def set_next_goal(id:int):
     helper.next_point=id
     return 

@app.delete("/robot/goal/")
def delete_next_goal():
    helper.next_point=-1
    return 

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=3001)
    