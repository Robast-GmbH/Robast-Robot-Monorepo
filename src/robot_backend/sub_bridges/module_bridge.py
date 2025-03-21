from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros


class ModuleBridge(BaseBridge):
    DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
    DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"
    MODULE_STATE_UPDATE_MSG = "communication_interfaces/msg/StateFeedback"
    LIVING_DEVICES_MSG = "std_msgs/msg/String"

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
        self.__module_state_update_subscriber = self.create_subscriber(
            "/module_state_update",
            ModuleBridge.MODULE_STATE_UPDATE_MSG,
            self.__on_module_state_update,
        )
        self.__living_devices_subscriber = self.create_subscriber(
            "/living_devices",
            ModuleBridge.LIVING_DEVICES_MSG,
            self.__on_living_devices_update,
        )

        self.__drawer_tree_publisher = self.create_publisher(
            "/trigger_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__electric_drawer_tree_publisher = self.create_publisher(
            "/trigger_electric_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__partial_drawer_tree_publisher = self.create_publisher(
            "/trigger_partial_drawer_tree",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )
        self.__close_drawer_publisher = self.create_publisher(
            "/close_drawer",
            ModuleBridge.DRAWER_ADDRESS_MSG,
        )

    def get_living_devices(self) -> list | None:
        return self.context.get("living_devices")

    def get_submodule_state(self, module_id: int, submodule_id: int) -> dict:
        try:
            return {
                "status": "success",
                "data": self.context[f"{module_id}_{submodule_id}"],
                "is_alive": self.__is_alive(module_id),
            }
        except KeyError:
            return {"status": "failure", "data": None}

    def open_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False
        if not self.__is_alive(module_id):
            print("Module is not alive")
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
        if not self.__is_alive(module_id):
            print("Module is not alive")
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

    def __is_module_type(self, module_type: int, module_id: int) -> bool:
        return module_id >> ModuleBridge.MODULE_UNIQUE_ID_LENGTH == module_type

    def __validate_module_id(self, module_id: int) -> bool:
        return (
            module_id >> ModuleBridge.MODULE_UNIQUE_ID_LENGTH
        ) in ModuleBridge.MODULE_TYPES.values()

    def __is_alive(self, module_id: int) -> bool:
        living_devices = self.get_living_devices()
        if living_devices is None:
            return False
        return module_id in living_devices

    def __on_module_state_update(self, message: dict) -> None:
        module_id = message["drawer_address"]["module_id"]
        drawer_id = message["drawer_address"]["drawer_id"]
        self.context[f"{module_id}_{drawer_id}"] = message["state"]

    def __on_living_devices_update(self, message: dict) -> None:
        self.context["living_devices"] = [
            int(module_id) for module_id in message["data"].split(",")
        ]
