from user_system.user_repository import UserRepository
from pydantic_models.user import User
from models.periodic_timer import PeriodicTimer
from configs.url_config import ROBOT_NAME_TO_IP, ROBOT_API_PORT
import requests


class AuthSessionManager:
    """
    Singleton class for handling the authentication sessions of the users.
    """

    __instance = None
    __initialized = False

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(AuthSessionManager, cls).__new__(
                cls, *args, **kwargs
            )
        return cls.__instance

    def __init__(self) -> None:
        if not self.__initialized:
            self.__initialized = True
            self.__fleet_ip_config = ROBOT_NAME_TO_IP
            self.__robot_api_port = ROBOT_API_PORT
            self.__user_repository = UserRepository()
            self.__sessions: dict[str, User | None] = {}
            for key in ROBOT_NAME_TO_IP:
                self.__sessions[key] = None
            self.__start_nfc_polling()

    def __start_nfc_polling(self) -> None:
        for robot_name in self.__fleet_ip_config:

            def timer_cb(name=robot_name):
                self.__nfc_polling_callback(robot_name=name)

            timer = PeriodicTimer(
                1,
                timer_cb,
            )
            timer.start()

    def __nfc_polling_callback(self, robot_name: str) -> None:
        try:
            response = requests.get(
                f"http://{self.__fleet_ip_config[robot_name]}:{self.__robot_api_port}/nfc_tag"
            )
            if response.status_code == 200:
                nfc_id = response.json()["nfc_tag"]["data"]
                user = self.__user_repository.get_user(nfc_id)
                if user and user != self.__sessions[robot_name]:
                    self.__start_session(robot_name, user)
        except Exception as e:
            print(f"Warning: {e}")
            

    def __start_session(self, robot_name: str, user: User) -> None:
        print(f"start_session {robot_name} {user.first_name} {user.last_name}")
        self.__sessions[robot_name] = user

    def end_session(self, robot_name: str) -> None:
        self.__sessions[robot_name] = None

    def get_session(self, robot_name: str) -> User | None:
        return self.__sessions[robot_name]

    def check_auth_status(
        self, robot_name: str, user_ids: list[str], user_groups: list[str]
    ) -> bool:
        session = self.__sessions.get(robot_name)
        if session:
            return session.id in user_ids or any(
                group in session.user_groups for group in user_groups
            )
        else:
            return False
