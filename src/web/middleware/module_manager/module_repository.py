from typing import List
from db_models.submodule import Submodule
from pydantic_models.submodule_address import SubmoduleAddress
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

DB_PATH = "sqlite:///modules2.db"
engine = create_engine(DB_PATH)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


class ModuleRepository:
    """
    Singleton class for handling the database operations related to modules.
    """

    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(ModuleRepository, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__create_table()

    def __create_table(self) -> None:
        """Create users table if it doesn't exist"""
        Submodule.metadata.create_all(bind=engine)

    def create_submodule(self, submodule: Submodule) -> Submodule | None:
        session = SessionLocal()
        try:
            session.add(submodule)
            session.commit()
            session.refresh(submodule)
            return submodule
        finally:
            session.close()

    def read_submodule(self, submodule_address: SubmoduleAddress) -> Submodule | None:
        session = SessionLocal()
        try:
            return (
                session.query(Submodule)
                .filter(
                    Submodule.robot_name == submodule_address.robot_name,
                    Submodule.module_id == submodule_address.module_id,
                    Submodule.submodule_id == submodule_address.submodule_id,
                )
                .first()
            )
        finally:
            session.close()

    def read_robot_submodules(self, robot_name: str) -> List[Submodule]:
        session = SessionLocal()
        try:
            return (
                session.query(Submodule)
                .filter(Submodule.robot_name == robot_name)
                .all()
            )
        finally:
            session.close()

    def update_submodule(self, updated_submodule: Submodule) -> Submodule | None:
        session = SessionLocal()
        try:
            submodule = (
                session.query(Submodule)
                .filter(
                    Submodule.robot_name == updated_submodule.robot_name,
                    Submodule.module_id == updated_submodule.module_id,
                    Submodule.submodule_id == updated_submodule.submodule_id,
                )
                .first()
            )
            if submodule:
                submodule.update_data(updated_submodule)
                session.commit()
                session.refresh(submodule)
                return submodule
        finally:
            session.close()
        return None

    def delete_submodule(self, submodule_address: SubmoduleAddress) -> bool:
        submodule = self.read_submodule(submodule_address)
        if submodule:
            try:
                session = SessionLocal()
                session.delete(submodule)
                session.commit()
            finally:
                session.close()
            return True
        return False
