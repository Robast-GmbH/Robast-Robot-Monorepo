"""
start module process needs external signal,
auth periodic timer,
wait for opening periodic timer,
opening external signal -> open_submodule call,
open periodic timer polling module is open,
closing external signal -> close_submodule call,
closed periodic timer polling module is closed,
idle external signal -> finish module process

stall_guard_triggered-> when stall guard has been triggered
opening_timed_out -> when manual drawer is not opened in time
"""

from functools import partial
from pydantic_models.submodule_address import SubmoduleAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_repository import ModuleRepository
from user_system.auth_session_manager import AuthSessionManager
from pydantic_models.submodule_process_request import SubmoduleProcessRequest
from pydantic_models.submodule_status import SubmoduleStatus
from db_models.submodule import Submodule
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
        self.__trigger_periodic_module_status_polling()

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
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/open_submodule?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        if submodule.module_process_status == "waiting_for_opening":
            submodule.module_process_status = "opening"
            self.repository.update_submodule(submodule)
        return True

    def close_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if not submodule:
            return False
        requests.post(
            f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}/close_submodule?module_id={address.module_id}&submodule_id={address.submodule_id}"
        )
        return True

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

    def __poll_submodule_status(self, address: SubmoduleAddress) -> None | Submodule:
        try:
            robot_base_url = f"http://{self.fleet_ip_config[address.robot_name]}:{self.robot_api_port}"
            response = requests.get(
                f"{robot_base_url}/submodule_status?module_id={address.module_id}&submodule_id={address.submodule_id}"
            )
            if response.status_code == 200 and response.json()["status"] == "success":
                submodule_status = response.json()["data"]
                return self.__update_module_process_status(address, submodule_status)
        except Exception as e:
            print(e)

    def __update_module_process_status(
        self, address: SubmoduleAddress, status: str
    ) -> None | Submodule:
        submodule = self.repository.read_submodule(address)
        if submodule:
            submodule.module_process_status = status
            return self.repository.update_submodule(submodule)

    def __trigger_periodic_module_status_polling(self) -> None:
        for robot in self.fleet_ip_config.keys():
            for submodule in self.repository.read_robot_submodules(robot):
                if submodule.module_process_status not in ["idle", "auth", "waiting_for_opening"]:
                    self.__poll_submodule_status(
                        SubmoduleAddress(
                            robot_name=robot,
                            module_id=submodule.module_id,
                            submodule_id=submodule.submodule_id,
                        )
                    )
        Timer(0.1, self.__trigger_periodic_module_status_polling).start()
