from os import read
import sqlite3
import uuid
from user_system.models.user import User

DB_PATH = "users.db"
AVAILABLE_USER_GROUPS = [
    "admin",
    "staff",
    "patient",
]


class UserRepository:
    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(UserRepository, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self, db_path: str = DB_PATH) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.db_path = db_path
            self.__create_table()

    def auth_user(self, user_id: str, nfc_id: str) -> bool:
        user = self.get_user(user_id)
        if user and user.nfc_id == nfc_id:
            return True
        return False

    def create_user(
        self,
        nfc_id: str,
        first_name: str,
        last_name: str,
        user_groups: list[str],
    ) -> User | None:
        user_id = str(uuid.uuid4())
        user_groups_str = ",".join(user_groups)
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            INSERT INTO users (user_id, nfc_id, first_name, last_name, user_groups) VALUES (?, ?, ?, ?, ?)
        """,
            (user_id, nfc_id, first_name, last_name, user_groups_str),
        )
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id)

    def get_all_users(self) -> list[User]:
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users")
        rows = cursor.fetchall()
        db_connection.close()
        users = []
        for row in rows:
            user_id, nfc_id, first_name, last_name, user_groups_str = row
            user_groups = user_groups_str.split(",")
            users.append(
                User(
                    id=user_id,
                    nfc_id=nfc_id,
                    first_name=first_name,
                    last_name=last_name,
                    user_groups=user_groups,
                )
            )
        return users

    def get_user(self, user_id: str) -> User | None:
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users WHERE user_id = ?", (user_id,))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            user_id, nfc_id, first_name, last_name, user_groups_str = row
            user_groups = user_groups_str.split(",")
            return User(
                id=user_id,
                nfc_id=nfc_id,
                first_name=first_name,
                last_name=last_name,
                user_groups=user_groups,
            )
        return None

    def get_user_by_nfc_id(self, nfc_id: str) -> User | None:
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users WHERE nfc_id = ?", (nfc_id,))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            user_id, nfc_id, first_name, last_name, user_groups_str = row
            user_groups = user_groups_str.split(",")
            return User(
                id=user_id,
                nfc_id=nfc_id,
                first_name=first_name,
                last_name=last_name,
                user_groups=user_groups,
            )
        return None

    def update_user(
        self,
        user_id: str,
        nfc_id: str | None = None,
        first_name: str | None = None,
        last_name: str | None = None,
        user_groups: list[str] | None = None,
    ) -> User | None:
        if not any(
            [
                nfc_id,
                first_name,
                last_name,
                user_groups,
            ]
        ):
            return

        query = "UPDATE users SET "
        params = []

        if nfc_id:
            query += "nfc_id = ?, "
            params.append(nfc_id)

        if first_name:
            query += "first_name = ?, "
            params.append(first_name)

        if last_name:
            query += "last_name = ?, "
            params.append(last_name)

        if user_groups:
            query += "user_groups = ?, "
            params.append(",".join(user_groups))

        # Remove the last comma and space
        query = query[:-2]

        query += " WHERE user_id = ?"
        params.append(user_id)
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute(query, params)
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id)

    def delete_user(self, user_id: str) -> bool:
        db_connection = sqlite3.connect(self.db_path)
        cursor = db_connection.cursor()
        cursor.execute("DELETE FROM users WHERE user_id = ?", (user_id,))
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id) is None

    def __create_table(self) -> None:
        db_connection = sqlite3.connect(self.db_path)
        # Create users table if not exists
        cursor = db_connection.cursor()
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS users (
                user_id TEXT PRIMARY KEY,
                nfc_id TEXT,
                first_name TEXT,
                last_name TEXT,
                user_groups TEXT
            )
        """
        )
        db_connection.commit()
        db_connection.close()
