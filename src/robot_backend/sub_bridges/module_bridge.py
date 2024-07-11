from sub_bridges.base_bridge import BaseBridge
from models.drawer import Drawer, TYPE_ELECTRIC_DRAWER
from models.module_process_data import ModuleProcessData
from roslibpy import Ros
from typing import Dict, List, Any

DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"

MODULE_PROCESS_STATE_WAITING_FOR_OPENING_COMMAND = "waiting_for_opening_command"
MODULE_PROCESS_STATE_OPENING = "opening"
MODULE_PROCESS_STATE_OPEN = "open"
MODULE_PROCESS_STATE_CLOSING = "closing"
MODULE_PROCESS_STATE_CLOSED = "closed"
MODULE_PROCESS_STATE_FINISHED = "finished"


class ModuleBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        Drawer.load_drawers("/workspace/src/robot_backend/configs/module_config.yaml")
        self.__drawer_open_subscriber = self.start_subscriber(
            "/bt_drawer_open",
            DRAWER_STATUS_MSG,
            on_msg_callback=self.__on_drawer_is_open_msg_callback,
        )
        self.__drawer_tree_publisher = self.start_publisher(
            "/trigger_drawer_tree",
            DRAWER_ADDRESS_MSG,
        )
        self.__electric_drawer_tree_publisher = self.start_publisher(
            "/trigger_electric_drawer_tree",
            DRAWER_ADDRESS_MSG,
        )
        self.__close_drawer_publisher = self.start_publisher(
            "/close_drawer",
            DRAWER_ADDRESS_MSG,
        )

        self.__current_module_process: Dict[str, Any] = {}

    def open_drawer(self, module_id: int, drawer_id: int) -> bool:
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            print("Module not found")
            return False

        self.__update_module_process_state(MODULE_PROCESS_STATE_OPENING)

        if drawer.get_type() == TYPE_ELECTRIC_DRAWER:
            self.__electric_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
        else:
            self.__drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
        return True

    def close_drawer(self, module_id: int, drawer_id: int) -> bool:
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            print("Module not found")
            return False
        if drawer.get_type() == TYPE_ELECTRIC_DRAWER:
            self.__close_drawer_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
            self.__update_module_process_state(MODULE_PROCESS_STATE_CLOSING)
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    def start_module_process(self, module_process_data: ModuleProcessData) -> str:
        current_module_process_state = self.__current_module_process.get("state")
        if (
            current_module_process_state is None
            or current_module_process_state == MODULE_PROCESS_STATE_FINISHED
        ):
            self.__current_module_process = {
                "module_id": module_process_data.module_id,
                "drawer_id": module_process_data.drawer_id,
                "process_name": module_process_data.process_name,
                "payload": module_process_data.payload,
                "state": MODULE_PROCESS_STATE_WAITING_FOR_OPENING_COMMAND,
            }
            return "Module process started."
        return "Module process has to be in finished state to start a new one."

    def finish_module_process(self) -> str:
        if self.__current_module_process["state"] != MODULE_PROCESS_STATE_CLOSED:
            return "Module process has to be in closed state to be finished."
        else:
            self.__update_module_process_state(MODULE_PROCESS_STATE_FINISHED)
            return "Module process finished."

    def get_modules(self) -> List[Dict[str, Any]]:
        return Drawer.drawers_as_json()

    def get_current_module_process(self) -> dict:
        return self.__current_module_process

    def __on_drawer_is_open_msg_callback(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        drawer_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]

        Drawer.get_drawer(module_id, drawer_id).set_is_open(is_open)

        self.__update_module_process_state(
            MODULE_PROCESS_STATE_OPEN if is_open else MODULE_PROCESS_STATE_CLOSED
        )

    def __update_module_process_state(self, state: str) -> None:
        current_state = self.__current_module_process.get("state")
        if current_state is not None and current_state != MODULE_PROCESS_STATE_FINISHED:
            self.__current_module_process["state"] = state
