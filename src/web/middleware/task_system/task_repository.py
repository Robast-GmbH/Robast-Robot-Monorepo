import copy
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, joinedload
from db_models.task import (
    Base,
    Task,
)
from db_models.subtask import Subtask


DB_PATH = "sqlite:///tasks2.db"
engine = create_engine(DB_PATH)
SessionLocal = sessionmaker(bind=engine)


class TaskRepository:
    """
    Singleton class for handling the database operations related to tasks using SQLAlchemy.
    """

    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(TaskRepository, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__create_tables()

    def __create_tables(self) -> None:
        """Create tasks and subtasks table if they don't exist"""
        Base.metadata.create_all(bind=engine)

    def create_task(self, task: Task) -> bool:
        session = SessionLocal()
        try:
            session.add(task)
            session.commit()
            return True
        except Exception as e:
            print(f"Error in create_task: {e}")
            session.rollback()
            return False
        finally:
            session.close()

    def read_tasks(self) -> list[Task]:
        session = SessionLocal()
        try:
            tasks = session.query(Task).options(joinedload(Task.subtasks)).all()
            return copy.deepcopy(tasks)
        finally:
            session.close()

    def read_task(self, task_id: str) -> Task | None:
        session = SessionLocal()
        try:
            task = (
                session.query(Task)
                .options(joinedload(Task.subtasks))
                .filter(Task.id == task_id)
                .first()
            )
            return copy.deepcopy(task)
        finally:
            session.close()

    def read_tasks_by_assignee(
        self, assignee_name: str, limit: int, offset: int
    ) -> list[Task]:
        session = SessionLocal()
        try:
            tasks = (
                session.query(Task)
                .options(joinedload(Task.subtasks))
                .filter(Task.assignee_name == assignee_name)
                .limit(limit)
                .offset(offset)
                .all()
            )
            return copy.deepcopy(tasks)
        finally:
            session.close()

    def read_finished_tasks_by_assignee(
        self, assignee_name: str, limit: int, offset: int
    ) -> list[Task]:
        session = SessionLocal()
        try:
            tasks = (
                session.query(Task)
                .options(joinedload(Task.subtasks))
                .filter(Task.assignee_name == assignee_name, Task.status == "finished")
                .order_by(Task.earliest_start_time.desc())
                .limit(limit)
                .offset(offset)
                .all()
            )
            return copy.deepcopy(tasks)
        finally:
            session.close()

    def read_unassigned_tasks(self) -> list[Task]:
        session = SessionLocal()
        try:
            tasks = (
                session.query(Task)
                .options(joinedload(Task.subtasks))
                .filter(Task.status == "unassigned")
                .order_by(Task.earliest_start_time)
                .all()
            )
            return copy.deepcopy(tasks)
        finally:
            session.close()

    def read_subtask(self, subtask_id: str) -> Subtask | None:
        session = SessionLocal()
        try:
            subtask = session.query(Subtask).filter(Subtask.id == subtask_id).first()
            return copy.deepcopy(subtask)
        finally:
            session.close()

    def read_subtasks_by_task_id(self, task_id: str) -> list[Subtask]:
        session = SessionLocal()
        try:
            subtasks = session.query(Subtask).filter(Subtask.parent_id == task_id).all()
            return subtasks
        finally:
            session.close()

    def read_subtasks_by_subtask_ids(self, subtask_ids: list[str]) -> list[Subtask]:
        session = SessionLocal()
        try:
            subtasks = session.query(Subtask).filter(Subtask.id.in_(subtask_ids)).all()
            return subtasks
        finally:
            session.close()

    def update_task(self, task: Task) -> bool:
        session = SessionLocal()
        try:
            session.merge(task)
            session.commit()
            return True
        except Exception as e:
            print(f"Error in update_task: {e}")
            session.rollback()
            return False
        finally:
            session.close()

    def update_subtask(self, subtask: Subtask) -> bool:
        session = SessionLocal()
        try:
            session.merge(subtask)
            session.commit()
            return True
        except Exception as e:
            print(f"Error in update_subtask: {e}")
            session.rollback()
            return False
        finally:
            session.close()

    def delete_task(self, task_id: str) -> bool:
        session = SessionLocal()
        try:
            session.query(Subtask).filter(Subtask.parent_id == task_id).delete()
            session.query(Task).filter(Task.id == task_id).delete()
            session.commit()
            return True
        except Exception as e:
            print(f"Error in delete_task: {e}")
            session.rollback()
            return False
        finally:
            session.close()

    def delete_subtask(self, subtask_id: str) -> bool:
        session = SessionLocal()
        try:
            session.query(Subtask).filter(Subtask.id == subtask_id).delete()
            session.commit()
            return True
        except Exception as e:
            print(f"Error in delete_subtask: {e}")
            session.rollback()
            return False
        finally:
            session.close()
