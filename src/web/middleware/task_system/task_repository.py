import json
import sqlite3
from pydantic_models.task import Task
from pydantic_models.sub_task import SubTask
from pydantic_models.action import Action

DB_PATH = "tasks.db"


class TaskRepository:
    """
    Singleton class for handling the database operations related to tasks.
    """

    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(TaskRepository, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self, db_path: str = DB_PATH) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__db_path = db_path
            self.__create_tables()

    def create_task(self, task: Task) -> int | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        is_successful = False
        try:
            cursor.execute(
                """
                INSERT INTO tasks (id, name, status, assignee_name, requirements, is_monolithic, earliest_start_time, priority)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    task.id,
                    task.name,
                    task.status,
                    task.assignee_name,
                    json.dumps(task.requirements),
                    task.is_monolithic,
                    task.earliest_start_time,
                    task.priority,
                ),
            )
            db_connection.commit()
            for subtask in task.subtasks:
                self.__create_subtask(cursor, subtask, task.id)
                db_connection.commit()
            is_successful = True
        except Exception as e:
            print(f"Warning ct: {e}")
        finally:
            db_connection.close()
        return is_successful

    def __create_subtask(self, cursor, subtask: SubTask, parent_id: str) -> bool:
        is_successful = False
        try:
            cursor.execute(
                """
                INSERT INTO subtasks (id, name, status, assignee_name, parent_id, requires_task_id, is_part_of_monolith, target_id, priority, earliest_start_time, requirements, action)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    subtask.id,
                    subtask.name,
                    subtask.status,
                    subtask.assignee_name,
                    parent_id,
                    subtask.requires_task_id,
                    subtask.is_part_of_monolith,
                    subtask.target_id,
                    subtask.priority,
                    subtask.earliest_start_time,
                    json.dumps(subtask.requirements),
                    json.dumps(subtask.action.to_json()) if subtask.action else None,
                ),
            )
            is_successful = True
        except Exception as e:
            print(f"Warning cst: {e}")
        return is_successful

    def read_task(self, task_id: str) -> Task | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM tasks WHERE id = ?
        """,
            (task_id,),
        )
        result = cursor.fetchone()
        db_connection.close()
        if result is None:
            return None
        return Task.from_database_row(
            result,
            self.read_subtasks_by_task_id(result[0]),
        )

    def read_tasks_by_assignee(self, assignee_name: str) -> list[Task]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM tasks WHERE assignee_name = ?
        """,
            (assignee_name,),
        )
        results = cursor.fetchall()
        db_connection.close()
        return [
            Task.from_database_row(
                result,
                self.read_subtasks_by_task_id(result[0]),
            )
            for result in results
        ]

    def read_subtask(self, subtask_id: str) -> SubTask | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM subtasks WHERE id = ?
        """,
            (subtask_id,),
        )
        result = cursor.fetchone()
        db_connection.close()
        if result is None:
            return None
        return SubTask.from_database_row(result)

    def read_unassigned_tasks(self) -> list[Task]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM tasks
            WHERE status = 'unassigned'
            ORDER BY earliest_start_time
        """
        )
        results = cursor.fetchall()
        db_connection.close()

        return [
            Task.from_database_row(
                result,
                self.read_subtasks_by_task_id(result[0]),
            )
            for result in results
        ]

    def read_tasks(self) -> list[Task]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM tasks
        """
        )
        results = cursor.fetchall()
        db_connection.close()
        return [
            Task.from_database_row(
                result,
                self.read_subtasks_by_task_id(result[0]),
            )
            for result in results
        ]

    def read_subtasks_by_task_id(self, task_id: str) -> list[SubTask]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM subtasks WHERE parent_id = ?
        """,
            (task_id,),
        )
        results = cursor.fetchall()
        db_connection.close()
        return [SubTask.from_database_row(result) for result in results]

    def read_subtasks_by_subtask_ids(self, subtask_ids: list[str]) -> list[SubTask]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            SELECT * FROM subtasks WHERE id IN ({})
        """.format(
                ",".join(["?"] * len(subtask_ids))
            ),
            subtask_ids,
        )
        results = cursor.fetchall()
        db_connection.close()
        return [SubTask.from_database_row(result) for result in results]

    def update_task(self, task: Task) -> bool:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        is_successful = False
        try:
            cursor.execute(
                """
                UPDATE tasks 
                SET name = ?, status = ?, assignee_name = ?, requirements = ?, is_monolithic = ?, earliest_start_time = ?, priority = ?
                WHERE id = ?
            """,
                (
                    task.name,
                    task.status,
                    task.assignee_name,
                    json.dumps(task.requirements),
                    task.is_monolithic,
                    task.earliest_start_time,
                    task.priority,
                    task.id,
                ),
            )
            db_connection.commit()
            db_connection.close()
            for subtask in task.subtasks:
                self.update_subtask(subtask)
            is_successful = True
        except Exception as e:
            print(f"Warning ut: {e}")
        return is_successful

    def update_subtask(self, subtask: SubTask) -> bool:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        is_successful = False
        try:
            cursor.execute(
                """
                UPDATE subtasks 
                SET name = ?, status = ?, assignee_name = ?, parent_id = ?, requires_task_id = ?, is_part_of_monolith = ?, target_id = ?, priority = ?, earliest_start_time = ?, requirements = ?, action = ?
                WHERE id = ?
            """,
                (
                    subtask.name,
                    subtask.status,
                    subtask.assignee_name,
                    subtask.parent_id,
                    subtask.requires_task_id,
                    subtask.is_part_of_monolith,
                    subtask.target_id,
                    subtask.priority,
                    subtask.earliest_start_time,
                    json.dumps(subtask.requirements),
                    json.dumps(subtask.action.to_json()),
                    subtask.id,
                ),
            )
            is_successful = True
        except Exception as e:
            print(f"Warning ust: {e}")
        finally:
            db_connection.commit()
            db_connection.close()
        return is_successful

    def delete_task(self, task_id: str) -> bool:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        is_successful = False
        try:
            cursor.execute(
                """
                DELETE FROM tasks WHERE id = ?
                """,
                (task_id,),
            )
            cursor.execute(
                """
                DELETE FROM subtasks WHERE parent_id = ?
                """,
                (task_id,),
            )
            is_successful = True
        except Exception as e:
            print(f"Warning dt: {e}")
        finally:
            db_connection.commit()
            db_connection.close()
        return is_successful

    def __create_tables(self) -> None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS tasks (
                id TEXT PRIMARY KEY,
                name TEXT,
                status TEXT,
                assignee_name TEXT,
                requirements TEXT,
                is_monolithic INTEGER,
                earliest_start_time INTEGER,
                priority INTEGER
            )
            """
        )

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS subtasks (
                id TEXT PRIMARY KEY,
                name TEXT,
                status TEXT,
                assignee_name TEXT,
                parent_id TEXT,
                requires_task_id TEXT,
                is_part_of_monolith INTEGER,
                target_id TEXT,
                priority INTEGER,
                earliest_start_time INTEGER,
                requirements TEXT,
                action TEXT,
                FOREIGN KEY (parent_id) REFERENCES tasks(id)
            )
            """
        )

        db_connection.commit()
        db_connection.close()
