import sqlite3
import json
from typing import List
from pydantic_models.submodule import Submodule
from pydantic_models.submodule_address import SubmoduleAddress

DB_PATH = "modules.db"


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

    def __init__(self, db_path: str = DB_PATH) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__db_path = db_path
            self.__create_table()

    def create_submodule(self, submodule: Submodule) -> int | None:
        sql = """
        INSERT INTO submodules (robot_name, module_id, submodule_id, position, size, variant, module_process_status, module_process_type, module_process_items_by_change, items_by_count, reserved_for_task, reserved_for_ids, reserved_for_groups)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        try:
            cursor.execute(
                sql,
                (
                    submodule.address.robot_name,
                    submodule.address.module_id,
                    submodule.address.submodule_id,
                    submodule.position,
                    submodule.size,
                    submodule.variant,
                    submodule.module_process_status,
                    submodule.module_process_type,
                    json.dumps(submodule.module_process_items_by_change),
                    json.dumps(submodule.items_by_count),
                    submodule.reserved_for_task,
                    json.dumps(submodule.reserved_for_ids),
                    json.dumps(submodule.reserved_for_groups),
                ),
            )
        except Exception as e:
            print(f"Warning: {e}")
            return None
        finally:
            db_connection.commit()
            db_connection.close()
        return cursor.lastrowid

    def read_submodule(self, address: SubmoduleAddress) -> Submodule | None:
        sql = "SELECT * FROM submodules WHERE robot_name = ? AND module_id = ? AND submodule_id = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            sql, (address.robot_name, address.module_id, address.submodule_id)
        )
        row = cursor.fetchone()
        db_connection.close()
        if row:
            return Submodule(
                address=SubmoduleAddress(
                    robot_name=row[0],
                    module_id=row[1],
                    submodule_id=row[2],
                ),
                position=row[3],
                size=row[4],
                variant=row[5],
                module_process_status=row[6],
                module_process_type=row[7],
                module_process_items_by_change=json.loads(row[8]),
                items_by_count=json.loads(row[9]),
                reserved_for_task=row[10],
                reserved_for_ids=json.loads(row[11]),
                reserved_for_groups=json.loads(row[12]),
            )
        return None

    def read_robot_submodules(self, robot_name: str) -> List[Submodule]:
        sql = "SELECT * FROM submodules WHERE robot_name = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(sql, (robot_name,))
        rows = cursor.fetchall()
        db_connection.close()
        submodules = []
        for row in rows:
            submodules.append(
                Submodule(
                    address=SubmoduleAddress(
                        robot_name=row[0],
                        module_id=row[1],
                        submodule_id=row[2],
                    ),
                    position=row[3],
                    size=row[4],
                    variant=row[5],
                    module_process_status=row[6],
                    module_process_type=row[7],
                    module_process_items_by_change=json.loads(row[8]),
                    items_by_count=json.loads(row[9]),
                    reserved_for_task=row[10],
                    reserved_for_ids=json.loads(row[11]),
                    reserved_for_groups=json.loads(row[12]),
                )
            )
        return submodules

    def update_submodule(self, submodule: Submodule):
        sql = """
        UPDATE submodules
        SET position = ?, size = ?, variant = ?, items_by_count = ?, reserved_for_task = ?, reserved_for_ids = ?, reserved_for_groups = ?, module_process_status = ?, module_process_type = ?, module_process_items_by_change = ?
        WHERE robot_name = ? AND module_id = ? AND submodule_id = ?
        """
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            sql,
            (
                submodule.position,
                submodule.size,
                submodule.variant,
                json.dumps(submodule.items_by_count),
                submodule.reserved_for_task,
                json.dumps(submodule.reserved_for_ids),
                json.dumps(submodule.reserved_for_groups),
                submodule.module_process_status,
                submodule.module_process_type,
                json.dumps(submodule.module_process_items_by_change),
                submodule.address.robot_name,
                submodule.address.module_id,
                submodule.address.submodule_id,
            ),
        )
        db_connection.commit()
        db_connection.close()

    def delete_submodule(self, address: SubmoduleAddress):
        sql = "DELETE FROM submodules WHERE robot_name = ? AND module_id = ? AND submodule_id = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            sql,
            (
                address.robot_name,
                address.module_id,
                address.submodule_id,
            ),
        )
        db_connection.commit()
        db_connection.close()

    def __create_table(self):
        create_table_sql = """
        CREATE TABLE IF NOT EXISTS submodules (
            robot_name TEXT NOT NULL,
            module_id INTEGER NOT NULL,
            submodule_id INTEGER NOT NULL,
            position INTEGER,
            size INTEGER,
            variant TEXT,
            module_process_status TEXT,
            module_process_type TEXT,
            module_process_items_by_change TEXT,
            items_by_count TEXT,
            reserved_for_task TEXT,
            reserved_for_ids TEXT,
            reserved_for_groups TEXT,
            PRIMARY KEY (robot_name, module_id, submodule_id)
        );
        """
        try:
            db_connection = sqlite3.connect(self.__db_path)
            cursor = db_connection.cursor()
            cursor.execute(create_table_sql)
            db_connection.close()
        except sqlite3.Error as e:
            print(e)
