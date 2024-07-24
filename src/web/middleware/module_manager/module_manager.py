from module_manager.module_repository import ModuleRepository
from pydantic_models.drawer import Drawer
from pydantic_models.drawer_address import DrawerAddress
from typing import Any


class ModuleManager:
    def __init__(self):
        self.repository = ModuleRepository()

    def is_module_size_mounted(self, robot_name: str, size: int) -> bool:
        drawers = self.repository.read_robot_drawers(robot_name)
        return any(drawer.size == size for drawer in drawers)

    def is_module_size_available(self, robot_name: str, size: int) -> bool:
        drawers = self.repository.read_robot_drawers(robot_name)
        return any(
            drawer.size == size
            and not drawer.reserved_for_ids
            and not drawer.reserved_for_groups
            for drawer in drawers
        )

    def get_modules(self, robot_name: str) -> list[dict[str, Any]]:
        drawers = self.repository.read_robot_drawers(robot_name)
        return [drawer.to_json() for drawer in drawers]

    def create_module(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
        position: int,
        size: int,
        variant: str,
    ) -> bool:
        drawer = Drawer(
            address=DrawerAddress(
                robot_name=robot_name,
                module_id=module_id,
                drawer_id=drawer_id,
            ),
            position=position,
            size=size,
            variant=variant,
            module_process_status="idle",
            module_process_type="",
            module_process_payload={},
            content={},
            reserved_for_ids=[],
            reserved_for_groups=[],
        )
        return self.repository.create_drawer(drawer) is not None

    def delete_module(self, address: DrawerAddress) -> bool:
        drawer = self.repository.read_drawer(address)
        if drawer:
            self.repository.delete_drawer(address)
            return True
        return False

    def update_module(self, address: DrawerAddress, variant: str | None) -> bool:
        drawer = self.repository.read_drawer(address)
        if drawer and variant:
            drawer.variant = variant
            self.repository.update_drawer(drawer)
            return True
        return False

    def empty_module(self, address: DrawerAddress) -> bool:
        drawer = self.repository.read_drawer(address)
        if drawer:
            drawer.content = {}
            self.repository.update_drawer(drawer)
            return True
        return False

    def update_module_content(
        self, address: DrawerAddress, content: dict[str, int]
    ) -> bool:
        drawer = self.repository.read_drawer(address)
        if drawer:
            for item_id, quantity in content.items():
                if quantity <= 0:
                    drawer.content.pop(item_id, None)
                else:
                    drawer.content[item_id] = quantity
                self.repository.update_drawer(drawer)
            return True
        return False

    def free_module(self, address: DrawerAddress) -> bool:
        return self.__update_module_reservation(address, [], [])

    def try_reserve_module_type(
        self, robot_name: str, size: int, user_ids: list[str], user_groups: list[str]
    ) -> Drawer | None:
        drawers = self.repository.read_robot_drawers(robot_name)
        for drawer in drawers:
            if (
                drawer.size == size
                and not drawer.reserved_for_ids
                and not drawer.reserved_for_groups
            ):
                self.reserve_module(
                    drawer.address,
                    user_ids,
                    user_groups,
                )
                return drawer

    def reserve_module(
        self, drawer_address: DrawerAddress, user_ids: list[str], user_groups: list[str]
    ) -> bool:
        return self.__update_module_reservation(
            drawer_address,
            user_ids,
            user_groups,
        )

    def __update_module_reservation(
        self, address: DrawerAddress, user_ids: list[str], user_groups: list[str]
    ) -> bool:
        drawer = self.repository.read_drawer(address)
        if drawer:
            drawer.reserved_for_ids = user_ids
            drawer.reserved_for_groups = user_groups
            self.repository.update_drawer(drawer)
            return True
        return False
