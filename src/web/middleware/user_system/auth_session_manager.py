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

    def try_start_session(
        self,
        robot_name: str,
        required_user_ids: list[str],
        required_user_groups: list[str],
    ) -> dict[str, str]:
        try:
            # Fetch NFC tag data from the robot's API
            response = requests.get(
                f"http://{self.__fleet_ip_config[robot_name]}:{self.__robot_api_port}/read_nfc_tag?timeout_in_s=30"
            )

            # Check if the request was successful
            if response.status_code == 200:
                nfc_id = response.json().get("nfc_tag", {}).get("data")
                if not nfc_id:
                    return {"status": "error", "message": "No NFC tag data in response"}

                user = self.__user_repository.get_user_by_nfc_id(nfc_id)
                if not user:
                    return {"status": "error", "message": "User not found"}

                # Check if the user is authorized
                if user.id in required_user_ids or any(
                    group in user.user_groups for group in required_user_groups
                ):
                    print(
                        f"start_session {robot_name} {user.first_name} {user.last_name}"
                    )
                    self.__sessions[robot_name] = user
                    return {"status": "success", "message": "Session started"}
                else:
                    return {"status": "error", "message": "User not authorized"}

            return {"status": "error", "message": "No NFC tag found"}

        except Exception as e:
            print(f"Warning: {e}")
            return {"status": "error", "message": str(e)}

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
