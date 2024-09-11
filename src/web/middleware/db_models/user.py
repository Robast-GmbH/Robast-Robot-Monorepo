from sqlalchemy.orm import DeclarativeBase
from sqlalchemy import Column, String, JSON
from typing import List


class Base(DeclarativeBase):
    pass


class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True)
    nfc_id = Column(String, index=True, unique=True, nullable=True)
    mail = Column(String, index=True, unique=True, nullable=True)
    title = Column(String)
    first_name = Column(String)
    last_name = Column(String)
    station = Column(String)
    room = Column(String)
    user_groups: List[str] = Column(JSON)
    password = Column(String)
