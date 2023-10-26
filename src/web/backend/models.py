from sqlalchemy import Boolean, Column, ForeignKey, Integer, String, Enum, Float, UniqueConstraint
from schemas import ActionType, DrawerSlideTypes
from database import Base


class User(Base):
    __tablename__ = "user"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, unique=True)
    full_name = Column(String, index=True, default="Unknown")
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    is_active = Column(Boolean, default=True)
    admin = Column(Boolean, default=False)


class Authentication(Base):
    __tablename__ = "authentication"
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("user.id"))
    nfc_code = Column(String, unique=True)
    enabled = Column(Boolean, default=True)


class Task(Base):
    __tablename__ = "task"
    id = Column(Integer, primary_key=True, index=True)
    robot_name = Column(String)
    fleet_name = Column(String)
    owner_id = Column(Integer, ForeignKey("user.id"))


class Action(Base):
    __tablename__ = "action"
    id = Column(Integer, primary_key=True, index=True)
    task_id = Column(Integer, ForeignKey("task.id"))
    finished = Column(Boolean, default=False)
    step = Column(Integer)
    type = Column(Enum(ActionType))
    status = Column(String)


class NavigationAction(Base):
    __tablename__ = "navigationaction"
    id = Column(Integer, ForeignKey('action.id'), primary_key=True)
    x_pose = Column(Float)
    y_pose = Column(Float)
    yaw_pose = Column(Float)


class DrawerAction(Base):
    __tablename__ = "draweraction"
    id = Column(Integer, ForeignKey('action.id'), primary_key=True)
    # target_user= Column(Integer, ForeignKey('user.id'))
    unit_id = Column(Integer, ForeignKey('module.unit_id'))


class DrawerLock(Base):
    __tablename__ = "drawerlock"
    id = Column(Integer, primary_key=True, index=True)
    active = Column(Boolean)
    user_id = Column(Integer, ForeignKey('user.id'))
    unit_id = Column(Integer, ForeignKey('module.unit_id'))


class NewUserAction(Base):
    __tablename__ = "useraction"
    id = Column(Integer, ForeignKey('action.id'), primary_key=True)
    user_id = Column(Integer, ForeignKey('user.id'))


class Module(Base):
    __tablename__ = "module"
    unit_id = Column(Integer, primary_key=True, index=True)
    module_id = Column(Integer)
    drawer_id = Column(Integer)
    position = Column(Integer)
    status = Column(String)
    type = Column(Enum(DrawerSlideTypes), index=True)
    size = Column(Integer)
    robot_name = Column(String)
    label = Column(String,
                   default="")
    __table_args__ = (UniqueConstraint("module_id",
                                       "drawer_id",
                                       name="module_module_drawer_unique"),)


class Robot(Base):
    __tablename__ = "robot"
    robot_name = Column(String, primary_key=True)
    fleet_name = Column(String)
    x_pose = Column(Float)
    y_pose = Column(Float)
    yaw_pose = Column(Float)
    task_id = Column(Integer, ForeignKey("task.id"))
    battery_level = Column(Float, default=0.0)
