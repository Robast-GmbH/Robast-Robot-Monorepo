"""
start module process needs external signal,
auth periodic timer,
wait for opening periodic timer,
opening external signal -> open_drawer call,
open periodic timer polling module is open,
closing external signal -> close_drawer call,
closed periodic timer polling module is closed,
idle external signal -> finish module process,
"""

from functools import partial
from pydantic_models.drawer_address import DrawerAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_repository import ModuleRepository
from user_system.auth_session_manager import AuthSessionManager
from pydantic_models.module_process_request import ModuleProcessRequest
from configs.url_config import ROBOT_NAME_TO_IP, ROBOT_API_PORT
from threading import Timer
import requests


class ModuleProcessManager:
    def __init__(
        self,
    ):
        self.auth_session_manager = AuthSessionManager()
        self.fleet_ip_config = ROBOT_NAME_TO_IP
        self.robot_api_port = ROBOT_API_PORT
        self.repository = ModuleRepository()
        self.module_manager = ModuleManager()

    def start_module_process(self, module_process_data: ModuleProcessRequest) -> bool:
        drawer_address = module_process_data.drawer_address
        drawer = self.repository.read_drawer(drawer_address)
        if not drawer:
            return False
        drawer.module_process_type = module_process_data.process_name
        drawer.module_process_payload = module_process_data.payload
        is_auth_required = drawer.reserved_for_ids or drawer.reserved_for_groups
        is_authenticated = self.auth_session_manager.check_auth_status(
            drawer_address.robot_name,
            drawer.reserved_for_ids,
            drawer.reserved_for_groups,
        )
        if is_auth_required and not is_authenticated:
            drawer.module_process_status = "auth"
            self.repository.update_drawer(drawer)
            self.wait_for_auth(drawer_address)
        else:
            drawer.module_process_status = "waiting_for_opening"
            self.repository.update_drawer(drawer)
        return True

    def get_module_process_status(self, address: DrawerAddress) -> str:
        drawer = self.repository.read_drawer(address)
        if drawer:
            return drawer.module_process_status
        else:
            return "not_found"

    def wait_for_auth(self, address: DrawerAddress) -> None:
        drawer = self.repository.read_drawer(address)
        if drawer:
            is_authenticated = self.auth_session_manager.check_auth_status(
                address.robot_name, drawer.reserved_for_ids, drawer.reserved_for_groups
            )
            if is_authenticated:
                drawer.module_process_status = "waiting_for_opening"
                self.repository.update_drawer(drawer)
            else:
                timer_cb = partial(self.wait_for_auth, address)
                Timer(0.5, timer_cb).start()

    def open_drawer(self, address: DrawerAddress) -> bool:
        drawer = self.repository.read_drawer(address)
        if not drawer:
            return False
        if (
            drawer.module_process_status != "waiting_for_opening"
            and drawer.module_process_status != "closed"
        ):
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/open_drawer?module_id={address.module_id}&drawer_id={address.drawer_id}"
        )
        drawer.module_process_status = "opening"
        self.repository.update_drawer(drawer)
        self.wait_drawer_open(address)
        return True

    def wait_drawer_open(self, address: DrawerAddress) -> None:
        is_open = self.poll_drawer_status(address)
        if is_open:
            drawer = self.repository.read_drawer(address)
            if drawer:
                drawer.module_process_status = "open"
                self.repository.update_drawer(drawer)
                self.wait_drawer_closed(address)
        else:
            timer_cb = partial(self.wait_drawer_open, address)
            Timer(0.5, timer_cb).start()

    def close_drawer(self, address: DrawerAddress) -> bool:
        drawer = self.repository.read_drawer(address)
        if not drawer:
            return False
        if drawer.module_process_status != "open":
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/close_drawer?module_id={address.module_id}&drawer_id={address.drawer_id}"
        )
        drawer.module_process_status = "closing"
        self.repository.update_drawer(drawer)
        self.wait_drawer_closed(address)
        return True

    def wait_drawer_closed(self, address: DrawerAddress) -> None:
        is_open = self.poll_drawer_status(address)
        if not is_open:
            drawer = self.repository.read_drawer(address)
            if drawer:
                drawer.module_process_status = "closed"
                self.repository.update_drawer(drawer)
        else:
            timer_cb = partial(self.wait_drawer_closed, address)
            Timer(0.5, timer_cb).start()

    def finish_module_process(self, address: DrawerAddress) -> bool:
        drawer = self.repository.read_drawer(address)
        if not drawer:
            return False
        for key, value in drawer.module_process_payload.items():
            if key in drawer.content:
                drawer.content[key] = drawer.content[key] + value
            else:
                drawer.content[key] = value
            if drawer.content[key] <= 0:
                drawer.content.pop(key)
        drawer.module_process_payload = {}
        drawer.module_process_type = ""
        drawer.module_process_status = "idle"
        if not drawer.content:
            drawer.reserved_for_ids = []
            drawer.reserved_for_groups = []
        self.repository.update_drawer(drawer)
        return True

    def poll_drawer_status(self, address: DrawerAddress) -> bool:
        robot_base_url = (
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}"
        )
        response = requests.get(
            f"{robot_base_url}/is_drawer_open?module_id={address.module_id}&drawer_id={address.drawer_id}"
        )
        if response.status_code == 200:
            return response.json()["is_open"]
        else:
            return False
