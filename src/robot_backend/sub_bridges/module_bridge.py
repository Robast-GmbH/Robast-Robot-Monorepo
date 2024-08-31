from sub_bridges.base_bridge import BaseBridge
from models.submodule import Submodule, TYPE_ELECTRIC_DRAWER
from roslibpy import Ros
from typing import Dict, List, Any

DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"


class ModuleBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        Submodule.load_submodules(
            "/release/src/robot_backend/configs/module_config.yaml"
        )
        self.__drawer_open_subscriber = self.start_subscriber(
            "/bt_drawer_open",
            DRAWER_STATUS_MSG,
            on_msg_callback=self.__on_submodule_is_open_msg_callback,
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

    def open_submodule(self, module_id: int, submodule_id: int) -> bool:
        submodule = Submodule.get_submodule(module_id, submodule_id)
        if submodule is None:
            print("Module not found")
            return False

        if submodule.get_type() == TYPE_ELECTRIC_DRAWER:
            self.__electric_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        else:
            self.__drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        return True

    def close_submodule(self, module_id: int, submodule_id: int) -> bool:
        submodule = Submodule.get_submodule(module_id, submodule_id)
        if submodule is None:
            print("Submodule not found")
            return False
        if submodule.get_type() == TYPE_ELECTRIC_DRAWER:
            self.__close_drawer_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    def get_submodules(self) -> List[Dict[str, Any]]:
        return Submodule.submodules_as_json()

    def get_submodule_is_open(self, module_id: int, submodule_id: int) -> bool:
        submodule = Submodule.get_submodule(module_id, submodule_id)
        if submodule is None:
            return False
        return submodule.get_is_open()

    def __on_submodule_is_open_msg_callback(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        submodule_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]

        Submodule.get_submodule(module_id, submodule_id).set_is_open(is_open)
