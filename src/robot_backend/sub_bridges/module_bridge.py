from sub_bridges.base_bridge import BaseBridge
from models.drawer import Drawer, TYPE_ELECTRIC_DRAWER
from roslibpy import Ros
from typing import Dict, List, Any

DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"

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

    def open_drawer(self, module_id: int, drawer_id: int) -> bool:
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            print("Module not found")
            return False

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
            return True
        else:
            print("Tried closing a manual drawer.")
            return False


    def get_modules(self) -> List[Dict[str, Any]]:
        return Drawer.drawers_as_json()

    def get_drawer_is_open(self, module_id: int, drawer_id: int) -> bool:
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            return False
        return drawer.get_is_open()

    def __on_drawer_is_open_msg_callback(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        drawer_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]

        Drawer.get_drawer(module_id, drawer_id).set_is_open(is_open)
