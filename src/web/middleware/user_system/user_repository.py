import uuid
import datetime
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from db_models.user import User


DB_URL = "sqlite:///users2.db"  # Using SQLite for demonstration, but you can change to PostgreSQL or others
AVAILABLE_USER_GROUPS = ["admin", "staff", "patient"]

# Setting up the SQLAlchemy engine and session
engine = create_engine(DB_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


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

    def __init__(self) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__create_table()

    def __create_table(self) -> None:
        """Create users table if it doesn't exist"""
        User.metadata.create_all(bind=engine)

    def get_session(self):
        """Helper function to create a new session"""
        return SessionLocal()

    def login_user(self, email: str, password: str) -> User | None:
        session = self.get_session()
        try:
            user = session.query(User).filter(User.mail == email).first()
            if user and user.password == password:
                return user
            return None
        finally:
            session.close()

    def create_user(
        self,
        nfc_id: str | None,
        mail: str | None,
        title: str,
        first_name: str,
        last_name: str,
        station: str,
        room: str,
        user_groups: list[str],
    ) -> User | None:
        session = self.get_session()
        user_id = str(uuid.uuid4())
        password = f"{first_name}-{last_name}-{datetime.datetime.now().year}"

        new_user = User(
            id=user_id,
            nfc_id=nfc_id,
            mail=mail,
            title=title,
            first_name=first_name,
            last_name=last_name,
            station=station,
            room=room,
            user_groups=user_groups,
            password=password,
        )

        session.add(new_user)
        session.commit()
        session.refresh(new_user)
        session.close()
        return new_user

    def get_all_users(self) -> list[User]:
        session = self.get_session()
        try:
            users = session.query(User).all()
            return users
        finally:
            session.close()

    def get_user(self, user_id: str) -> User | None:
        session = self.get_session()
        try:
            user = session.query(User).filter(User.id == user_id).first()
            return user
        finally:
            session.close()

    def get_user_by_nfc_id(self, nfc_id: str) -> User | None:
        session = self.get_session()
        try:
            user = session.query(User).filter(User.nfc_id == nfc_id).first()
            return user
        finally:
            session.close()

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
        session = self.get_session()
        try:
            user = session.query(User).filter(User.id == user_id).first()
            if user:
                if nfc_id:
                    user.nfc_id = nfc_id
                if mail:
                    user.mail = mail
                if title is not None:
                    user.title = title
                if first_name:
                    user.first_name = first_name
                if last_name:
                    user.last_name = last_name
                if station:
                    user.station = station
                if room:
                    user.room = room
                if user_groups:
                    user.user_groups = user_groups

                session.commit()
                session.refresh(user)
                return user
            return None
        finally:
            session.close()

    def update_user_password(
        self, user_id: str, old_password: str, new_password: str
    ) -> bool:
        session = self.get_session()
        try:
            user = (
                session.query(User)
                .filter(User.id == user_id, User.password == old_password)
                .first()
            )
            if user:
                user.password = new_password
                session.commit()
                return True
            return False
        finally:
            session.close()

    def delete_user(self, user_id: str) -> bool:
        session = self.get_session()
        try:
            user = session.query(User).filter(User.id == user_id).first()
            if user:
                session.delete(user)
                session.commit()
                return True
            return False
        finally:
            session.close()
