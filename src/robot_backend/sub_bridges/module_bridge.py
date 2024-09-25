from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros


class ModuleBridge(BaseBridge):
    DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
    DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"
    E_DRAWER_STATUS_MSG = "communication_interfaces/msg/ElectricalDrawerStatus"

    MODULE_UNIQUE_ID_LENGTH = 16
    MODULE_TYPES = {
        "MANUAL_DRAWER_10x40": 0b00010001,
        "MANUAL_DRAWER_20x40": 0b00010010,
        "MANUAL_DRAWER_30x40": 0b00010011,
        "E_DRAWER_10x40": 0b00010100,
        "PARTIAL_DRAWER_10x40x8": 0b00010101,
        "DINNER_TRAYS": 0b00010110,
        "SURGERY_TOOLS": 0b00010111,
    }

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.__drawer_open_subscriber = self.start_subscriber(
            "/bt_drawer_open",
            ModuleBridge.DRAWER_STATUS_MSG,
            on_msg_callback=self.__on_submodule_is_open_msg,
        )
        self.__e_drawer_status_subscriber = self.start_subscriber(
            "/electrical_drawer_status",
            ModuleBridge.E_DRAWER_STATUS_MSG,
            on_msg_callback=self.__on_electrical_drawer_status_msg,
        )
        self.__drawer_tree_publisher = self.start_publisher(
            "/trigger_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__electric_drawer_tree_publisher = self.start_publisher(
            "/trigger_electric_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__partial_drawer_tree_publisher = self.start_publisher(
            "/trigger_partial_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__close_drawer_publisher = self.start_publisher(
            "/close_drawer",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )

    def open_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False

        if self.__is_module_type(
            module_type=ModuleBridge.MODULE_TYPES["E_DRAWER_10x40"],
            module_id=module_id,
        ):
            self.__electric_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        elif self.__is_module_type(
            module_type=ModuleBridge.MODULE_TYPES["PARTIAL_DRAWER_10x40x8"],
            module_id=module_id,
        ):
            self.__partial_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        else:
            self.__drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        return True

    def close_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False
        if self.__is_module_type(
            module_type=ModuleBridge.MODULE_TYPES["E_DRAWER_10x40"],
            module_id=module_id,
        ) or self.__is_module_type(
            module_type=ModuleBridge.MODULE_TYPES["PARTIAL_DRAWER_10x40x8"],
            module_id=module_id,
        ):
            self.__close_drawer_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    def get_submodule_is_open(self, module_id: int, submodule_id: int) -> bool:
        try:
            return self.context[f"{str(module_id)}_{str(submodule_id)}"]["is_open"]
        except KeyError:
            return False

    def get_submodule_stall_guard_triggered(
        self, module_id: int, submodule_id: int
    ) -> bool:
        try:
            return self.context[f"{str(module_id)}_{str(submodule_id)}_stall_guard_triggered"]
        except KeyError:
            return False

    def __on_submodule_is_open_msg(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        submodule_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]
        self.context[f"{str(module_id)}_{str(submodule_id)}"] = {"is_open": is_open}

    def __is_module_type(self, module_type: int, module_id: int) -> bool:
        return module_id >> ModuleBridge.MODULE_UNIQUE_ID_LENGTH == module_type

    def __validate_module_id(self, module_id: int) -> bool:
        return (
            module_id >> ModuleBridge.MODULE_UNIQUE_ID_LENGTH
        ) in ModuleBridge.MODULE_TYPES.values()

    def __on_electrical_drawer_status_msg(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        drawer_id = msg["drawer_address"]["drawer_id"]
        is_stall_guard_triggered = msg["is_stall_guard_triggered"]
        self.context[f"{str(module_id)}_{str(drawer_id)}_stall_guard_triggered"] = (
            is_stall_guard_triggered
        )
