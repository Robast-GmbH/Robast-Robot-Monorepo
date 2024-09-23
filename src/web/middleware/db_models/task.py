from sqlalchemy import Boolean, Column, Integer, String, JSON
from sqlalchemy.orm import DeclarativeBase, relationship
from typing import Dict, Any


class Base(DeclarativeBase):
    pass


class Task(Base):
    __tablename__ = "tasks"

    id = Column(String, primary_key=True)
    name = Column(String)
    status = Column(String)
    assignee_name = Column(String)
    requirements: Dict[str, Any] = Column(JSON)
    is_monolithic = Column(Boolean)
    earliest_start_time = Column(Integer)
    priority = Column(Integer)

    subtasks = relationship("Subtask", back_populates="parent")

    def to_json(self):
        return {
            "id": self.id,
            "name": self.name,
            "status": self.status,
            "assignee_name": self.assignee_name,
            "requirements": self.requirements if self.requirements else {},
            "is_monolithic": self.is_monolithic,
            "earliest_start_time": self.earliest_start_time,
            "priority": self.priority,
            "subtasks": [subtask.to_json() for subtask in self.subtasks],
        }
