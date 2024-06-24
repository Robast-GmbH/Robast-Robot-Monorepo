# start module process needs external signal
# auth periodic timer
# wait for opening periodic timer
# opening external signal -> open_drawer call
# open periodic timer polling module is open
# closing external signal -> close_drawer call
# closed periodic timer polling module is closed
# idle external signal -> finish module process
from module_manager.module_manager import ModuleManager
from module_manager.module_repository import ModuleRepository
from user_system.auth_session_manager import AuthSessionManager
from pydantic_models.module_process_data import ModuleProcessData
from configs.url_config import robot_name_to_ip, robot_api_port
from threading import Timer
import requests


class ModuleProcessManager:
    def __init__(
        self,
    ):
        self.auth_session_manager = AuthSessionManager()
        self.fleet_ip_config = robot_name_to_ip
        self.robot_api_port = robot_api_port
        self.repository = ModuleRepository()
        self.module_manager = ModuleManager()

    def start_module_process(
        self,
        robot_name: str,
        module_process_data: ModuleProcessData,
    ) -> bool:
        drawer = self.repository.read_drawer(
            robot_name, module_process_data.module_id, module_process_data.drawer_id
        )
        if not drawer:
            return False
        drawer.module_process_type = module_process_data.process_name
        drawer.module_process_payload = module_process_data.payload
        is_auth_required = drawer.reserved_for_ids or drawer.reserved_for_groups
        is_authenicated = self.auth_session_manager.check_auth_status(
            robot_name, drawer.reserved_for_ids, drawer.reserved_for_groups
        )
        if is_auth_required and not is_authenicated:
            drawer.module_process_status = "auth"
            self.repository.update_drawer(drawer)
            self.wait_for_auth(
                robot_name,
                module_process_data.module_id,
                module_process_data.drawer_id,
            )
        else:
            drawer.module_process_status = "waiting_for_opening"
            self.repository.update_drawer(drawer)
        return True

    def get_module_process_status(
        self, robot_name: str, module_id: int, drawer_id: int
    ) -> str:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            return drawer.module_process_status
        else:
            return "not_found"

    def wait_for_auth(self, robot_name: str, module_id: int, drawer_id: int) -> None:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            is_authenticated = self.auth_session_manager.check_auth_status(
                robot_name, drawer.reserved_for_ids, drawer.reserved_for_groups
            )
            if is_authenticated:
                drawer.module_process_status = "waiting_for_opening"
                self.repository.update_drawer(drawer)
            else:
                Timer(
                    0.5, self.wait_for_auth, args=(robot_name, module_id, drawer_id)
                ).start()

    def open_drawer(self, robot_name: str, module_id: int, drawer_id: int) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if not drawer:
            return False
        if (
            drawer.module_process_status != "waiting_for_opening"
            and drawer.module_process_status != "closed"
        ):
            return False
        requests.post(
            f"http://{self.fleet_ip_config[robot_name]}:{self.robot_api_port}/open_drawer?module_id={module_id}&drawer_id={drawer_id}"
        )
        drawer.module_process_status = "opening"
        self.repository.update_drawer(drawer)
        self.wait_drawer_open(robot_name, module_id, drawer_id)
        return True

    def wait_drawer_open(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
    ) -> None:
        is_open = self.poll_drawer_status(robot_name, module_id, drawer_id)
        if is_open:
            drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
            if drawer:
                drawer.module_process_status = "open"
                self.repository.update_drawer(drawer)
                self.wait_drawer_closed(robot_name, module_id, drawer_id)
        else:
            Timer(
                0.5, self.wait_drawer_open, args=(robot_name, module_id, drawer_id)
            ).start()

    def close_drawer(self, robot_name: str, module_id: int, drawer_id: int) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if not drawer:
            return False
        if drawer.module_process_status != "open":
            return False
        requests.post(
            f"http://{self.fleet_ip_config[robot_name]}:{self.robot_api_port}/close_drawer?module_id={module_id}&drawer_id={drawer_id}"
        )
        drawer.module_process_status = "closing"
        self.repository.update_drawer(drawer)
        self.wait_drawer_closed(robot_name, module_id, drawer_id)
        return True

    def wait_drawer_closed(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
    ) -> None:
        is_open = self.poll_drawer_status(robot_name, module_id, drawer_id)
        if not is_open:
            drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
            if drawer:
                drawer.module_process_status = "closed"
                self.repository.update_drawer(drawer)
        else:
            Timer(
                0.5, self.wait_drawer_closed, args=(robot_name, module_id, drawer_id)
            ).start()

    def finish_module_process(
        self, robot_name: str, module_id: int, drawer_id: int
    ) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if not drawer:
            return False
        for key, value in drawer.module_process_payload.items():
            drawer.content[key] = value
        drawer.module_process_payload = {}
        drawer.module_process_type = ""
        drawer.module_process_status = "idle"
        self.repository.update_drawer(drawer)
        return True

    def poll_drawer_status(
        self, robot_name: str, module_id: int, drawer_id: int
    ) -> bool:
        robot_base_url = (
            f"http://{self.fleet_ip_config[robot_name]}:{self.robot_api_port}"
        )
        response = requests.get(
            f"{robot_base_url}/is_drawer_open?module_id={module_id}&drawer_id={drawer_id}"
        )
        if response.status_code == 200:
            return response.json()["is_open"]
        else:
            return False
