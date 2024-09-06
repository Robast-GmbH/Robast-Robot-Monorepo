from module_manager.module_repository import ModuleRepository
from pydantic_models.submodule import Submodule
from pydantic_models.submodule_address import SubmoduleAddress
from typing import Any


class ModuleManager:
    MODULE_TYPES = [
        0b0001,  # MANUAL_DRAWER_10x40
        0b0010,  # MANUAL_DRAWER_20x40
        0b0011,  # MANUAL_DRAWER_30x40
        0b0100,  # E_DRAWER_10x40
        0b0101,  # PARTIAL_DRAWER_10x40x8
        0b0110,  # DINNER_TRAYS
        0b0111,  # SURGERY_TOOLS
    ]
    MODULE_TYPE_BIT_MASK = 0x0F
    MODULE_UNIQUE_ID_LENGTH = 16

    def __init__(self):
        self.repository = ModuleRepository()

    def is_submodule_type_mounted(self, robot_name: str, size: int) -> bool:
        submodules = self.repository.read_robot_submodules(robot_name)
        return any(
            self.__is_size(size, submodule.address.module_id)
            for submodule in submodules
        )

    def is_submodule_size_available(self, robot_name: str, size: int) -> bool:
        submodules = self.repository.read_robot_submodules(robot_name)
        return any(
            self.__is_size(size, submodule.address.module_id)
            and not submodule.reserved_for_ids
            and not submodule.reserved_for_groups
            for submodule in submodules
        )

    def get_submodules(self, robot_name: str) -> list[dict[str, Any]]:
        submodules = self.repository.read_robot_submodules(robot_name)
        return [submodule.to_json() for submodule in submodules]

    def create_submodule(
        self,
        robot_name: str,
        module_id: int,
        submodule_id: int,
        position: int,
        size: int,
        variant: str,
    ) -> bool:
        submodule = Submodule(
            address=SubmoduleAddress(
                robot_name=robot_name,
                module_id=module_id,
                submodule_id=submodule_id,
            ),
            position=position,
            size=size,
            variant=variant,
            module_process_status="idle",
            module_process_type="",
            module_process_items_by_change={},
            items_by_count={},
            reserved_for_task="",
            reserved_for_ids=[],
            reserved_for_groups=[],
        )
        return self.repository.create_submodule(submodule) is not None

    def delete_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule:
            self.repository.delete_submodule(address)
            return True
        return False

    def update_submodule(self, address: SubmoduleAddress, variant: str | None) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule and variant:
            submodule.variant = variant
            self.repository.update_submodule(submodule)
            return True
        return False

    def empty_submodule(self, address: SubmoduleAddress) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule:
            submodule.items_by_count = {}
            self.repository.update_submodule(submodule)
            return True
        return False

    def update_submodule_content(
        self, address: SubmoduleAddress, items_by_count: dict[str, int]
    ) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule:
            for item_id, quantity in items_by_count.items():
                if quantity <= 0:
                    submodule.items_by_count.pop(item_id, None)
                else:
                    submodule.items_by_count[item_id] = quantity
                self.repository.update_submodule(submodule)
            return True
        return False

    def free_submodule(self, address: SubmoduleAddress) -> bool:
        return self.__update_submodule_reservation(address, "", [], [])

    def try_reserve_submodule_type(
        self,
        robot_name: str,
        size: int,
        task_id: str,
        user_ids: list[str],
        user_groups: list[str],
    ) -> Submodule | None:
        submodules = self.repository.read_robot_submodules(robot_name)
        for submodule in submodules:
            if (
                self.__is_size(size, submodule.address.module_id)
                and not submodule.reserved_for_task
                and not submodule.reserved_for_ids
                and not submodule.reserved_for_groups
            ):
                self.reserve_submodule(
                    submodule.address,
                    task_id,
                    user_ids,
                    user_groups,
                )
                return submodule

    def reserve_submodule(
        self,
        submodule_address: SubmoduleAddress,
        task_id: str,
        user_ids: list[str],
        user_groups: list[str],
    ) -> bool:
        return self.__update_submodule_reservation(
            submodule_address,
            task_id,
            user_ids,
            user_groups,
        )

    def __update_submodule_reservation(
        self,
        address: SubmoduleAddress,
        task_id: str,
        user_ids: list[str],
        user_groups: list[str],
    ) -> bool:
        submodule = self.repository.read_submodule(address)
        if submodule:
            submodule.reserved_for_task = task_id
            submodule.reserved_for_ids = user_ids
            submodule.reserved_for_groups = user_groups
            self.repository.update_submodule(submodule)
            return True
        return False

    def __is_module_type(self, module_type: int, module_id: int):
        extracted_module_type = (
            module_id >> self.MODULE_UNIQUE_ID_LENGTH
        ) & self.MODULE_TYPE_BIT_MASK
        return extracted_module_type == module_type

    def __is_size(self, size: int, module_id: int) -> bool:
        size_prefixes = self.__size_to_prefixes(size)
        for prefix in size_prefixes:
            if self.__is_module_type(prefix, module_id):
                return True
        return False

    def __size_to_prefixes(self, size: int) -> list[int]:
        if size == 1:
            return [0b0001, 0b0100]
        elif size == 2:
            return [0b0010]
        elif size == 3:
            return [0b0011]
        elif size == 5:
            return [0b0101]
        return []
