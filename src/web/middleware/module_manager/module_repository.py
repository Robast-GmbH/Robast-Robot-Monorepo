import sqlite3
import json
from typing import List
from module_manager.models.drawer import Drawer

DB_PATH = "modules.db"


class ModuleRepository:
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

    def create_drawer(self, drawer: Drawer) -> int | None:
        sql = """
        INSERT INTO drawers (robot_name, module_id, drawer_id, position, size, variant, module_process_status, module_process_type, module_process_payload, content, reserved_for_ids, reserved_for_groups)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        try:
            cursor.execute(
                sql,
                (
                    drawer.robot_name,
                    drawer.module_id,
                    drawer.drawer_id,
                    drawer.position,
                    drawer.size,
                    drawer.variant,
                    drawer.module_process_status,
                    drawer.module_process_type,
                    json.dumps(drawer.module_process_payload),
                    json.dumps(drawer.content),
                    json.dumps(drawer.reserved_for_ids),
                    json.dumps(drawer.reserved_for_groups),
                ),
            )
        except Exception as e:
            return None
        finally:
            db_connection.commit()
            db_connection.close()
        return cursor.lastrowid

    def read_drawer(
        self, robot_name: str, module_id: int, drawer_id: int
    ) -> Drawer | None:
        sql = "SELECT * FROM drawers WHERE robot_name = ? AND module_id = ? AND drawer_id = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(sql, (robot_name, module_id, drawer_id))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            return Drawer(
                robot_name=row[0],
                module_id=row[1],
                drawer_id=row[2],
                position=row[3],
                size=row[4],
                variant=row[5],
                module_process_status=row[6],
                module_process_type=row[7],
                module_process_payload=json.loads(row[8]),
                content=json.loads(row[9]),
                reserved_for_ids=json.loads(row[10]),
                reserved_for_groups=json.loads(row[11]),
            )
        return None

    def read_robot_drawers(self, robot_name: str) -> List[Drawer]:
        sql = "SELECT * FROM drawers WHERE robot_name = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(sql, (robot_name,))
        rows = cursor.fetchall()
        db_connection.close()
        drawers = []
        for row in rows:
            drawers.append(
                Drawer(
                    robot_name=row[0],
                    module_id=row[1],
                    drawer_id=row[2],
                    position=row[3],
                    size=row[4],
                    variant=row[5],
                    module_process_status=row[6],
                    module_process_type=row[7],
                    module_process_payload=json.loads(row[8]),
                    content=json.loads(row[9]),
                    reserved_for_ids=json.loads(row[10]),
                    reserved_for_groups=json.loads(row[11]),
                )
            )
        return drawers

    def update_drawer(self, drawer: Drawer):
        sql = """
        UPDATE drawers
        SET position = ?, size = ?, variant = ?, content = ?, reserved_for_ids = ?, reserved_for_groups = ?, module_process_status = ?, module_process_type = ?, module_process_payload = ?
        WHERE robot_name = ? AND module_id = ? AND drawer_id = ?
        """
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            sql,
            (
                drawer.position,
                drawer.size,
                drawer.variant,
                json.dumps(drawer.content),
                json.dumps(drawer.reserved_for_ids),
                json.dumps(drawer.reserved_for_groups),
                drawer.module_process_status,
                drawer.module_process_type,
                json.dumps(drawer.module_process_payload),
                drawer.robot_name,
                drawer.module_id,
                drawer.drawer_id,
            ),
        )
        db_connection.commit()
        db_connection.close()

    def delete_drawer(self, robot_name: str, module_id: int, drawer_id: int):
        sql = "DELETE FROM drawers WHERE robot_name = ? AND module_id = ? AND drawer_id = ?"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(sql, (robot_name, module_id, drawer_id))
        db_connection.commit()
        db_connection.close()

    def __create_table(self):
        create_table_sql = """
        CREATE TABLE IF NOT EXISTS drawers (
            robot_name TEXT NOT NULL,
            module_id INTEGER NOT NULL,
            drawer_id INTEGER NOT NULL,
            position INTEGER,
            size INTEGER,
            variant TEXT,
            module_process_status TEXT,
            module_process_type TEXT,
            module_process_payload TEXT,
            content TEXT,
            reserved_for_ids TEXT,
            reserved_for_groups TEXT,
            PRIMARY KEY (robot_name, module_id, drawer_id)
        );
        """
        try:
            db_connection = sqlite3.connect(self.__db_path)
            cursor = db_connection.cursor()
            cursor.execute(create_table_sql)
            db_connection.close()
        except sqlite3.Error as e:
            print(e)
