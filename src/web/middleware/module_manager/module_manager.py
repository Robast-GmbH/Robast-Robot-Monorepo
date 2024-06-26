from module_manager.module_repository import ModuleRepository, Drawer
from typing import Any


class ModuleManager:
    def __init__(self):
        self.repository = ModuleRepository()

    def is_module_size_available(self, robot_name: str, size: int) -> bool:
        drawers = self.repository.read_robot_drawers(robot_name)
        return any(drawer.size == size for drawer in drawers)

    def get_modules(self, robot_name: str) -> list[dict[str, Any]]:
        drawers = self.repository.read_robot_drawers(robot_name)
        return [drawer.to_json() for drawer in drawers]

    def create_module(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
        postion: int,
        size: int,
        variant: str,
    ) -> bool:
        drawer = Drawer(
            robot_name,
            module_id,
            drawer_id,
            postion,
            size,
            variant,
            "idle",
            "",
            {},
            {},
            [],
            [],
        )
        return self.repository.create_drawer(drawer) is not None

    def delete_module(self, robot_name: str, module_id: int, drawer_id: int) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            self.repository.delete_drawer(robot_name, module_id, drawer_id)
            return True
        return False

    def empty_module(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
    ) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            drawer.content = {}
            self.repository.update_drawer(drawer)
            return True
        return False

    def update_module_content(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
        item_id: str,
        quantity: int,
    ) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            drawer.content[item_id] = quantity
            self.repository.update_drawer(drawer)
            return True
        return False

    def free_module(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
    ) -> bool:
        return self.__update_module_reservation(
            robot_name, module_id, drawer_id, [], []
        )

    def try_reserve_module_type(
        self,
        robot_name: str,
        size: int,
        user_ids: list[str],
        user_groups: list[str],
    ) -> dict[str, Any] | None:
        drawers = self.repository.read_robot_drawers(robot_name)
        for drawer in drawers:
            if (
                drawer.size == size
                and not drawer.reserved_for_ids
                and not drawer.reserved_for_groups
            ):
                self.reserve_module(
                    robot_name,
                    drawer.module_id,
                    drawer.drawer_id,
                    user_ids,
                    user_groups,
                )
                return drawer.to_json()

    def reserve_module(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
        user_ids: list[str],
        user_groups: list[str],
    ) -> bool:
        return self.__update_module_reservation(
            robot_name, module_id, drawer_id, user_ids, user_groups
        )

    def __update_module_reservation(
        self,
        robot_name: str,
        module_id: int,
        drawer_id: int,
        user_ids: list[str],
        user_groups: list[str],
    ) -> bool:
        drawer = self.repository.read_drawer(robot_name, module_id, drawer_id)
        if drawer:
            drawer.reserved_for_ids = user_ids
            drawer.reserved_for_groups = user_groups
            self.repository.update_drawer(drawer)
            return True
        return False
