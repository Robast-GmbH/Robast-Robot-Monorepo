from enum import unique
from operator import index
from turtle import position
from xmlrpc.client import boolean
from sqlalchemy import Boolean, Column, ForeignKey, Integer, String, MetaData, Enum, Float, tuple_
from sqlalchemy.orm import relationship

from schemas import TaskTypes, DrawerTypes
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
    target_user_id =Column(Integer, default=0)
    drawer_id= Column(Integer, default= 0)
    robot_name= Column(String, default="rb0")
    Fleet_name= Column(String, default="ROBAST_1")
    owner_id = Column(Integer, ForeignKey("users.id"))
    owner = relationship("User", cascade="save-update")


class Drawer(Base):
    __tablename__ = "drawer"
    id = Column(Integer, primary_key=True, index=True)
    robot_drawer_id= Column(Integer)
    type= Column(Enum(DrawerTypes), index=True)
    robot= Column(String)
    fleet=Column(String)
    
