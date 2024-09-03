from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros

DRAWER_STATUS_MSG = "communication_interfaces/msg/DrawerStatus"
DRAWER_ADDRESS_MSG = "communication_interfaces/msg/DrawerAddress"


# example manual 0b00010001000000000000000000000001
# example electric 0b00010100000000000000000000000001
# 0b00010001000000000000000000000001 manual small
# 0b00010100000000000000000000000001 electric small
# 0b00010001000000000000000000000010 manual small
# 0b00010010000000000000000000000001 manual medium
# 0b00010011000000000000000000000001 manual large
# 0b00010101000000000000000000000001 electric partial
class ModuleBridge(BaseBridge):
    TRANSPORTATION_MODULE_ID_PREFIX = 0b0001
    ELECTRIC_DRAWER_MODULE_ID_PREFIX = 0b0100
    PARTIAL_DRAWER_MODULE_ID_PREFIX = 0b0101
    MODULE_ID_PREFIXES = [
        0b0001,  # MANUAL_DRAWER_10x40
        0b0010,  # MANUAL_DRAWER_20x40
        0b0011,  # MANUAL_DRAWER_30x40
        0b0100,  # E_DRAWER_10x40
        0b0101,  # PARTIAL_DRAWER_10x40x8
        0b0110,  # DINNER_TRAYS
        0b0111,  # SURGERY_TOOLS
    ]

    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
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
        self.__partial_drawer_tree_publisher = self.start_publisher(
            "/trigger_partial_drawer_tree",
            DRAWER_ADDRESS_MSG,
        )
        self.__close_drawer_publisher = self.start_publisher(
            "/close_drawer",
            DRAWER_ADDRESS_MSG,
        )

    def open_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False

        if self.__is_module_type(
            module_prefix=self.ELECTRIC_DRAWER_MODULE_ID_PREFIX,
            module_id=module_id,
        ):
            self.__electric_drawer_tree_publisher.publish(
                {
                    "module_id": self.__remove_bits_8_to_15_and_get_24bit(module_id),
                    "drawer_id": submodule_id,
                }
            )
        elif self.__is_module_type(
            module_prefix=self.PARTIAL_DRAWER_MODULE_ID_PREFIX, module_id=module_id
        ):
            self.__partial_drawer_tree_publisher.publish(
                {
                    "module_id": self.__remove_bits_8_to_15_and_get_24bit(module_id),
                    "drawer_id": submodule_id,
                }
            )
        else:
            self.__drawer_tree_publisher.publish(
                {
                    "module_id": self.__remove_bits_8_to_15_and_get_24bit(module_id),
                    "drawer_id": submodule_id,
                }
            )
        return True

    def close_submodule(self, module_id: int, submodule_id: int) -> bool:
        if not self.__validate_module_id(module_id):
            print("Invalid module id")
            return False

        if self.__is_module_type(
            module_prefix=self.ELECTRIC_DRAWER_MODULE_ID_PREFIX,
            module_id=module_id,
        ) or self.__is_module_type(
            module_prefix=self.PARTIAL_DRAWER_MODULE_ID_PREFIX, module_id=module_id
        ):
            self.__close_drawer_publisher.publish(
                {
                    "module_id": self.__remove_bits_8_to_15_and_get_24bit(module_id),
                    "drawer_id": submodule_id,
                }
            )
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    def get_submodule_is_open(self, module_id: int, submodule_id: int) -> bool:
        try:
            module_id = self.__remove_bits_8_to_15_and_get_24bit(module_id)
            return self.context[f"{str(module_id)}_{str(submodule_id)}"]["is_open"]
        except KeyError:
            return False

    def __on_submodule_is_open_msg_callback(self, msg: dict) -> None:
        module_id = msg["drawer_address"]["module_id"]
        submodule_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]
        self.context[f"{str(module_id)}_{str(submodule_id)}"] = {"is_open": is_open}

    def __is_module_type(self, module_prefix: int, module_id: int) -> bool:
        bitmask = 0b11111111000000000000000000000000
        complete_module_type_prefix = self.TRANSPORTATION_MODULE_ID_PREFIX << 4
        complete_module_type_prefix = complete_module_type_prefix | module_prefix
        prefix_shifted = complete_module_type_prefix << 24
        return (module_id & bitmask) == prefix_shifted

    def __validate_module_id(self, module_id: int) -> bool:
        # Combine prefix 0b0001 with each of the 4-bit values to form 8-bit values
        valid_prefixes = [
            (self.TRANSPORTATION_MODULE_ID_PREFIX << 4) | module_id_prefix
            for module_id_prefix in self.MODULE_ID_PREFIXES
        ]

        # Extract the first 8 bits of the input value
        first_8_bits = (module_id >> 24) & 0xFF

        return first_8_bits in valid_prefixes

    def __remove_bits_8_to_15_and_get_24bit(self, value):
        # Mask to remove bits 8 to 15
        lower_bits = value & 0xFF  # Keep bits 0 to 7
        upper_bits = (value >> 16) & 0xFFFF  # Keep bits 16 and above

        # Combine lower bits and shifted upper bits
        result = (upper_bits << 8) | lower_bits

        return result
