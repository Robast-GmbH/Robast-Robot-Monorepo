from typing import List, Optional, Any
from enum import Enum
import yaml

from pydantic import BaseModel


class OrderTypes(str, Enum):
    FOOD = "Food"
    MEDS = "Meds"
    OTHER = "others"



class GoalBase(BaseModel):
    x: int
    y: int
    clientX: Optional[float]
    clientY: Optional[float]


class Goal(GoalBase):
    pass
    class Config:
        orm_mode = True


class GoalCreate(GoalBase):
    pass


class OrderBase(BaseModel):
    order_item: str
    order_type: Optional[OrderTypes] = OrderTypes.FOOD
    goal: Goal
    recurring_order: bool
    finished: Optional[bool] = False
    description: Optional[str] = ""


class OrderCreate(OrderBase):
    pass


class Order(OrderBase):
    id: int
    owner_id: int

    class Config:
        orm_mode = True


class UserBase(BaseModel):
    name: str
    
class UserLogin(UserBase):
    hashed_password: str

class UserCreate(UserBase):
    hashed_password: str
    full_name: str
    email: str

class User(UserBase):
    id: int
    is_active: bool
    admin: bool
    full_name: str
    orders: List[Order] = []

    class Config:
        orm_mode = True
#drawer

class DrawerBase(BaseModel):
    drawer_controller_id: int
    content: str
    empty: bool
    
class DrawerCreate(DrawerBase):
    pass


class Drawer(DrawerBase):
   
    id: int

    class Config:
        orm_mode = True

#mappositions


class MapPositionBase(BaseModel):
    name: str
    x: float
    y: float
    t:float


class MapPositionCreate(MapPositionBase):
    pass

class MapPosition(MapPositionBase):
    id: int

    class Config:
        orm_mode = True

#mapping

class Room(BaseModel):
    x: int
    y: int
    name: str
    id: int


class RoomsBase(BaseModel):
    map_name: str
    map_yaml: dict


class RoomsCreate(RoomsBase):
    pass


class Rooms(RoomsBase):
    id: int
    rooms: List[Room] = []

    class Config:
        orm_mode = True