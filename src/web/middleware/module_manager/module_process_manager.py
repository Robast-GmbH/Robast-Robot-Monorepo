"""
start module process needs external signal,
auth periodic timer,
wait for opening periodic timer,
opening external signal -> open_submodule call,
open periodic timer polling module is open,
closing external signal -> close_submodule call,
closed periodic timer polling module is closed,
idle external signal -> finish module process

stall_guard_triggered_on_opening -> when stall guard is triggered while opening
stall_guard_triggered_on_closing -> when stall guard is triggered while closing
opening_timed_out -> when manual drawer is not opened in time
"""

from functools import partial
from pydantic_models.submodule_address import SubmoduleAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_repository import ModuleRepository
from user_system.auth_session_manager import AuthSessionManager
from pydantic_models.submodule_process_request import SubmoduleProcessRequest
from pydantic_models.submodule_status import SubmoduleStatus
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
        submodule.module_process_items_by_change = (
            submodule_process_request.items_by_change
        )
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
            and submodule.module_process_status != "opening_timed_out"
            and submodule.module_process_status != "stall_guard_triggered_on_opening"
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
        submodule_status = self.__poll_submodule_status(address)
        if not submodule_status:
            return
        if submodule_status.is_open:
            self.__update_module_process_status(address, "open")
            self.wait_submodule_closed(address)
        elif submodule_status.opening_timed_out:
            self.__update_module_process_status(address, "opening_timed_out")
        elif submodule_status.is_stall_guard_triggered:
            self.__update_module_process_status(
                address, "stall_guard_triggered_on_opening"
            )
        else:
            timer_cb = partial(self.wait_submodule_open, address)
            Timer(0.5, timer_cb).start()

    def close_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        if (
            submodule.module_process_status != "open"
            and submodule.module_process_status != "stall_guard_triggered_on_closing"
            and submodule.module_process_status != "stall_guard_triggered_on_opening"
        ):
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/close_submodule?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        submodule.module_process_status = "closing"
        self.repository.update_submodule(submodule)
        self.wait_submodule_closed(address)
        return True

    def wait_submodule_closed(self, address: SubmoduleAddress) -> None:
        submodule_status = self.__poll_submodule_status(address)
        if not submodule_status:
            return
        if not submodule_status.is_open:
            self.__update_module_process_status(address, "closed")
        elif submodule_status.is_stall_guard_triggered:
            self.__update_module_process_status(
                address, "stall_guard_triggered_on_closing"
            )
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

    def cancel_submodule_process(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        submodule.module_process_items_by_change = {}
        submodule.module_process_type = ""
        submodule.module_process_status = "idle"
        self.repository.update_submodule(submodule)
        return True

    def __poll_submodule_status(
        self, address: SubmoduleAddress
    ) -> SubmoduleStatus | None:
        robot_base_url = (
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}"
        )
        response = requests.get(
            f"{robot_base_url}/submodule_status?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        if response.status_code == 200:
            submodule_status = response.json()
            return SubmoduleStatus.from_json(submodule_status)
        return None

    def __update_module_process_status(
        self, address: SubmoduleAddress, status: str
    ) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule:
            submodule.module_process_status = status
            self.repository.update_submodule(submodule)
            return True
        return False
