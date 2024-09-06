from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros


class ModuleBridge(BaseBridge):
    DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
    DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"

    MODULE_UNIQUE_ID_LENGTH = 16
    ELECTRIC_DRAWER_MODULE_ID_PREFIX = 0b00010100
    PARTIAL_DRAWER_MODULE_ID_PREFIX = 0b00010101
    MODULE_TYPES = [
        0b00010001,  # MANUAL_DRAWER_10x40
        0b00010010,  # MANUAL_DRAWER_20x40
        0b00010011,  # MANUAL_DRAWER_30x40
        0b00010100,  # E_DRAWER_10x40
        0b00010101,  # PARTIAL_DRAWER_10x40x8
        0b00010110,  # DINNER_TRAYS
        0b00010111,  # SURGERY_TOOLS
    ]

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.__drawer_open_subscriber = self.start_subscriber(
            "/bt_drawer_open",
            self.DRAWER_STATUS_MSG,
            on_msg_callback=self.__on_submodule_is_open_msg_callback,
        )
        self.__drawer_tree_publisher = self.start_publisher(
            "/trigger_drawer_tree",
            self.DRAWER_ADDRESS_MSG,
        )
        self.__electric_drawer_tree_publisher = self.start_publisher(
            "/trigger_electric_drawer_tree",
            self.DRAWER_ADDRESS_MSG,
        )
        self.__partial_drawer_tree_publisher = self.start_publisher(
            "/trigger_partial_drawer_tree",
            self.DRAWER_ADDRESS_MSG,
        )
        self.__close_drawer_publisher = self.start_publisher(
            "/close_drawer",
            self.DRAWER_ADDRESS_MSG,
        )

    def open_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False

        if self.__is_module_type(
            module_type=self.ELECTRIC_DRAWER_MODULE_ID_PREFIX,
            module_id=module_id,
        ):
            self.__electric_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": submodule_id}
            )
        elif self.__is_module_type(
            module_type=self.PARTIAL_DRAWER_MODULE_ID_PREFIX, module_id=module_id
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
            module_type=self.ELECTRIC_DRAWER_MODULE_ID_PREFIX,
            module_id=module_id,
        ) or self.__is_module_type(
            module_type=self.PARTIAL_DRAWER_MODULE_ID_PREFIX, module_id=module_id
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

    def __on_submodule_is_open_msg_callback(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        submodule_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]
        self.context[f"{str(module_id)}_{str(submodule_id)}"] = {"is_open": is_open}

    def __is_module_type(self, module_type: int, module_id: int) -> bool:
        return module_id >> self.MODULE_UNIQUE_ID_LENGTH == module_type

    def __validate_module_id(self, module_id: int) -> bool:
        return (module_id >> self.MODULE_UNIQUE_ID_LENGTH) in self.MODULE_TYPES
