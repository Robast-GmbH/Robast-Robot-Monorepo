from enum import unique
from operator import index
from turtle import position
from xmlrpc.client import boolean
from sqlalchemy import Boolean, Column, ForeignKey, Integer, String, MetaData, Enum, Float, tuple_
from sqlalchemy.orm import relationship

from schemas import TaskTypes, DrawerSlideTypes
from database import Base



class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, unique=True)
    full_name= Column(String, index =True, default="Unknown" )
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    is_active = Column(Boolean, default=True)
    admin = Column(Boolean, default=False)


class Task(Base):
    __tablename__ = "task"
    id = Column(Integer, primary_key=True, index=True)
    task_type = Column(Enum(TaskTypes), index=True)
    x_pose = Column(Float)
    y_pose = Column(Float)
    yaw_pose= Column(Float)
    finished = Column(Boolean, default=False)
    target_id =Column(Integer, default=0)
    drawer_id= Column(Integer, default= 0)
    robot_name= Column(String, default="RB0")
    fleet_name= Column(String, default="ROBAST")
    owner_id = Column(Integer, ForeignKey("users.id"))
    owner = relationship("User", cascade="save-update")


class Module(Base):
    __tablename__ = "module"
    id = Column(Integer, primary_key=True)
    drawer_id= Column(Integer)
    type= Column(Enum(DrawerSlideTypes), index=True)
    size= Column(Integer)
    robot_name= Column(String)
    

class Robot(Base):
    __tablename__ = "robot"
    robot_name= Column(String, primary_key=True)
    fleet_name= Column(String)
    x_pose = Column(Float)
    y_pose = Column(Float)
    yaw_pose = Column(Float)
    task_id = Column(Integer)

    
