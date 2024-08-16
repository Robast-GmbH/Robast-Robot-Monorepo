"""
start module process needs external signal,
auth periodic timer,
wait for opening periodic timer,
opening external signal -> open_submodule call,
open periodic timer polling module is open,
closing external signal -> close_submodule call,
closed periodic timer polling module is closed,
idle external signal -> finish module process,
"""

from functools import partial
from pydantic_models.submodule_address import SubmoduleAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_repository import ModuleRepository
from user_system.auth_session_manager import AuthSessionManager
from pydantic_models.submodule_process_request import SubmoduleProcessRequest
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

    def start_submodule_process(
        self, submodule_process_request: SubmoduleProcessRequest
    ) -> bool:
        submodule_address = submodule_process_request.submodule_address
        submodule = self.repository.read_submodule(submodule_address)
        if not submodule:
            return False
        submodule.module_process_type = submodule_process_request.process_name
        submodule.module_process_items_by_change = submodule_process_request.items_by_change
        is_auth_required = submodule.reserved_for_ids or submodule.reserved_for_groups
        is_authenticated = self.auth_session_manager.check_auth_status(
            submodule_address.robot_name,
            submodule.reserved_for_ids,
            submodule.reserved_for_groups,
        )
        if is_auth_required and not is_authenticated:
            submodule.module_process_status = "auth"
            self.repository.update_submodule(submodule)
            self.wait_for_auth(submodule_address)
        else:
            submodule.module_process_status = "waiting_for_opening"
            self.repository.update_submodule(submodule)
        return True

    def get_submodule_process_status(self, address: SubmoduleAddress) -> str:
        submodule = self.repository.read_submodule(address)
        if submodule:
            return submodule.module_process_status
        else:
            return "not_found"

    def wait_for_auth(self, address: SubmoduleAddress) -> None:
        submodule = self.repository.read_submodule(address)
        if submodule:
            is_authenticated = self.auth_session_manager.check_auth_status(
                address.robot_name,
                submodule.reserved_for_ids,
                submodule.reserved_for_groups,
            )
            if is_authenticated:
                submodule.module_process_status = "waiting_for_opening"
                self.repository.update_submodule(submodule)
            else:
                timer_cb = partial(self.wait_for_auth, address)
                Timer(0.5, timer_cb).start()

    def open_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        if (
            submodule.module_process_status != "waiting_for_opening"
            and submodule.module_process_status != "closed"
        ):
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/open_submodule?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        submodule.module_process_status = "opening"
        self.repository.update_submodule(submodule)
        self.wait_submodule_open(address)
        return True

    def wait_submodule_open(self, address: SubmoduleAddress) -> None:
        is_open = self.poll_submodule_status(address)
        if is_open:
            submodule = self.repository.read_submodule(address)
            if submodule:
                submodule.module_process_status = "open"
                self.repository.update_submodule(submodule)
                self.wait_submodule_closed(address)
        else:
            timer_cb = partial(self.wait_submodule_open, address)
            Timer(0.5, timer_cb).start()

    def close_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        if submodule.module_process_status != "open":
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/close_submodule?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        submodule.module_process_status = "closing"
        self.repository.update_submodule(submodule)
        self.wait_submodule_closed(address)
        return True

    def wait_submodule_closed(self, address: SubmoduleAddress) -> None:
        is_open = self.poll_submodule_status(address)
        if not is_open:
            submodule = self.repository.read_submodule(address)
            if submodule:
                submodule.module_process_status = "closed"
                self.repository.update_submodule(submodule)
        else:
            timer_cb = partial(self.wait_submodule_closed, address)
            Timer(0.5, timer_cb).start()

    def finish_submodule_process(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        for key, value in submodule.module_process_items_by_change.items():
            if key in submodule.items_by_count:
                submodule.items_by_count[key] = submodule.items_by_count[key] + value
            else:
                submodule.items_by_count[key] = value
            if submodule.items_by_count[key] <= 0:
                submodule.items_by_count.pop(key)
        submodule.module_process_items_by_change = {}
        submodule.module_process_type = ""
        submodule.module_process_status = "idle"
        if not submodule.items_by_count:
            submodule.reserved_for_ids = []
            submodule.reserved_for_groups = []
        self.repository.update_submodule(submodule)
        return True

    def poll_submodule_status(self, address: SubmoduleAddress) -> bool:
        robot_base_url = (
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}"
        )
        response = requests.get(
            f"{robot_base_url}/is_submodule_open?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        if response.status_code == 200:
            return response.json()["is_open"]
        else:
            return False
