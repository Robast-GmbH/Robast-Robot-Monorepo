import datetime
import sqlite3
import uuid
from pydantic_models.user import User

DB_PATH = "users.db"
AVAILABLE_USER_GROUPS = [
    "admin",
    "staff",
    "patient",
]


class UserRepository:
    """
    Singleton class for handling the database operations related to users.
    """

    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(UserRepository, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def __init__(self, db_path: str = DB_PATH) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__db_path = db_path
            self.__create_table()

    def login_user(self, email: str, password: str) -> User | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users WHERE mail = ?", (email,))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            (
                user_id,
                nfc_id,
                mail,
                title,
                first_name,
                last_name,
                station,
                room,
                user_groups_str,
                user_password,
            ) = row
            if user_password == password:
                return User(
                    id=user_id,
                    nfc_id=nfc_id,
                    mail=mail,
                    title=title,
                    first_name=first_name,
                    last_name=last_name,
                    station=station,
                    room=room,
                    user_groups=user_groups_str.split(","),
                )
        return None

    def create_user(
        self,
        nfc_id: str,
        mail: str,
        title: str,
        first_name: str,
        last_name: str,
        station: str,
        room: str,
        user_groups: list[str],
    ) -> User | None:
        user_id = str(uuid.uuid4())
        user_groups_str = ",".join(user_groups)
        password = f"{first_name}-{last_name}-{datetime.datetime.now().year}"
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(
            """
            INSERT INTO users (user_id, nfc_id, mail, title, first_name, last_name, station, room, user_groups, password) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                user_id,
                nfc_id,
                mail,
                title,
                first_name,
                last_name,
                station,
                room,
                user_groups_str,
                password,
            ),
        )
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id)

    def get_all_users(self) -> list[User]:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users")
        rows = cursor.fetchall()
        db_connection.close()
        users = []
        for row in rows:
            (
                user_id,
                nfc_id,
                mail,
                title,
                first_name,
                last_name,
                station,
                room,
                user_groups_str,
                _,
            ) = row
            user_groups = user_groups_str.split(",")
            users.append(
                User(
                    id=user_id,
                    nfc_id=nfc_id,
                    mail=mail,
                    title=title,
                    first_name=first_name,
                    last_name=last_name,
                    station=station,
                    room=room,
                    user_groups=user_groups,
                )
            )
        return users

    def get_user(self, user_id: str) -> User | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users WHERE user_id = ?", (user_id,))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            (
                user_id,
                nfc_id,
                mail,
                title,
                first_name,
                last_name,
                station,
                room,
                user_groups_str,
                _,
            ) = row
            user_groups = user_groups_str.split(",")
            return User(
                id=user_id,
                nfc_id=nfc_id,
                mail=mail,
                title=title,
                first_name=first_name,
                last_name=last_name,
                station=station,
                room=room,
                user_groups=user_groups,
            )
        return None

    def get_user_by_nfc_id(self, nfc_id: str) -> User | None:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute("SELECT * FROM users WHERE nfc_id = ?", (nfc_id,))
        row = cursor.fetchone()
        db_connection.close()
        if row:
            (
                user_id,
                nfc_id,
                mail,
                title,
                first_name,
                last_name,
                station,
                room,
                user_groups_str,
                _,
            ) = row
            user_groups = user_groups_str.split(",")
            return User(
                id=user_id,
                nfc_id=nfc_id,
                mail=mail,
                title=title,
                first_name=first_name,
                last_name=last_name,
                station=station,
                room=room,
                user_groups=user_groups,
            )
        return None

    def update_user(
        self,
        user_id: str,
        nfc_id: str | None = None,
        mail: str | None = None,
        title: str | None = None,
        first_name: str | None = None,
        last_name: str | None = None,
        station: str | None = None,
        room: str | None = None,
        user_groups: list[str] | None = None,
    ) -> User | None:
        if not any(
            [
                nfc_id,
                mail,
                title is not None,
                first_name,
                last_name,
                station,
                room,
                user_groups,
            ]
        ):
            return

        query = "UPDATE users SET "
        params = []

        if nfc_id:
            query += "nfc_id = ?, "
            params.append(nfc_id)

        if mail:
            query += "mail = ?, "
            params.append(mail)

        if title is not None:
            query += "title = ?, "
            params.append(title)

        if first_name:
            query += "first_name = ?, "
            params.append(first_name)

        if last_name:
            query += "last_name = ?, "
            params.append(last_name)

        if station:
            query += "station = ?, "
            params.append(station)

        if room:
            query += "room = ?, "
            params.append(room)

        if user_groups:
            query += "user_groups = ?, "
            params.append(",".join(user_groups))

        # Remove the last comma and space
        query = query[:-2]

        query += " WHERE user_id = ?"
        params.append(user_id)
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute(query, params)
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id)

    def update_user_password(
        self, user_id: str, old_password: str, new_password: str
    ) -> bool:
        try:
            db_connection = sqlite3.connect(self.__db_path)
            cursor = db_connection.cursor()
            cursor.execute(
                "UPDATE users SET password = ? WHERE user_id = ? AND password = ?",
                (new_password, user_id, old_password),
            )
            if cursor.rowcount > 0:
                db_connection.commit()
                return True
            else:
                return False
        except sqlite3.Error:
            return False
        finally:
            db_connection.close()

    def delete_user(self, user_id: str) -> bool:
        db_connection = sqlite3.connect(self.__db_path)
        cursor = db_connection.cursor()
        cursor.execute("DELETE FROM users WHERE user_id = ?", (user_id,))
        db_connection.commit()
        db_connection.close()
        return self.get_user(user_id) is None

    def __create_table(self) -> None:
        db_connection = sqlite3.connect(self.__db_path)
        # Create users table if not exists
        cursor = db_connection.cursor()
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS users (
                user_id TEXT PRIMARY KEY,
                nfc_id TEXT,   
                mail TEXT,
                title TEXT,
                first_name TEXT,
                last_name TEXT,
                station TEXT,
                room TEXT,
                user_groups TEXT,
                password TEXT
            )
        """
        )
        db_connection.commit()
        db_connection.close()
