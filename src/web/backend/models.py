from enum import unique
from operator import index
from turtle import position
from xmlrpc.client import boolean
from sqlalchemy import Boolean, Column, ForeignKey, Integer, String, MetaData, Enum, Float, tuple_
from sqlalchemy.orm import relationship

from schemas import OrderTypes
from database import Base


class Goal(Base):
    __tablename__ = "goals"

    owner_id = Column(Integer, ForeignKey("orders.id"), primary_key=True)
    owner = relationship("Order", back_populates="goal")

    x = Column(Integer, index=True)
    y = Column(Integer, index=True)
    clientX = Column(Float)
    clientY = Column(Float)


class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, unique=True)
    full_name= Column(String, index =True, default="Unknown" )
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    is_active = Column(Boolean, default=True)
    admin = Column(Boolean, default=False)


class Order(Base):
    __tablename__ = "orders"

    id = Column(Integer, primary_key=True, index=True)
    order_item = Column(String, index=True)
    recurring_order = Column(Boolean, default=False)
    description = Column(String, index=True)
    order_type = Column(Enum(OrderTypes), index=True)
    finished = Column(Boolean, default=False)

    owner_id = Column(Integer, ForeignKey("users.id"))
    owner = relationship("User", cascade="save-update")

    goal = relationship("Goal", back_populates="owner", cascade="all, delete-orphan", uselist=False)


class Drawer(Base):
    __tablename__ = "drawer"
    id = Column(Integer, primary_key=True, index=True)
    drawer_controller_id= Column(Integer, unique=True)
    content = Column(String, unique=True, index= True)
    empty= Column(Boolean, default=False )
    
class MapPosition(Base):
    __tablename__="map_position"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, unique=True)
    x= Column(Float)
    y= Column(Float)

    t=Column(Float)
       

#mapping
class Room(Base):
    __tablename__ = "rooms"

    id = Column(Integer, primary_key=True)
    name = Column(String, index=True)
    x = Column(Integer, index=True)
    y = Column(Integer, index=True)

    owner_id = Column(Integer, ForeignKey("roomMap.id"))
    owner = relationship("RoomMap", cascade="save-update")


class RoomMap(Base):
    __tablename__ = "roomMap"

    id = Column(Integer, primary_key=True)
    name = Column(String, index=True)

    rooms = relationship("Room", back_populates="owner")


