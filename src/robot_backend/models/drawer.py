from __future__ import annotations
from typing import Dict, List, Any
from thread_safe_dict import ThreadSafeDict
import yaml

TYPE_MANUAL_DRAWER = "manual_drawer"
TYPE_ELECTRIC_DRAWER = "electric_drawer"


class Drawer:
    def __init__(
        self,
        module_id: int,
        drawer_id: int,
        pos: int,
        size: int,
        type: str,
        label: str,
    ) -> None:
        self.__module_id = module_id
        self.__drawer_id = drawer_id
        self.__pos = pos
        self.__size = size
        self.__type = type
        self.__label = label
        self.__is_open = False

    __instances: ThreadSafeDict[str, Drawer] = ThreadSafeDict()

    @classmethod
    def load_drawers(cls, path: str) -> None:
        with open(path, "r", encoding="UTF-8") as file:
            data = yaml.safe_load(file)
        for drawer in data["modules"]:
            cls.create_drawer_from_json(drawer)

    @classmethod
    def drawers_as_json(cls) -> List[Dict[str, Any]]:
        return [drawer.to_json() for drawer in cls.__instances.values()]

    @classmethod
    def get_drawer(cls, module_id: int, drawer_id: int) -> Drawer | None:
        concatenated_id = f"{module_id}_{drawer_id}"
        try:
            return cls.__instances[concatenated_id]
        except KeyError:
            return None

    @classmethod
    def create_drawer_from_json(cls, json: Dict[str, Any]) -> Drawer:
        drawer = Drawer(
            json["module_id"],
            json["drawer_id"],
            json["pos"],
            json["size"],
            json["type"],
            json["label"],
        )
        concatenated_id = f"{drawer.__module_id}_{drawer.__drawer_id}"
        cls.__instances[concatenated_id] = drawer

        return drawer

    def set_is_open(self, is_open: bool) -> None:
        self.__is_open = is_open

    def get_type(self) -> str:
        return self.__type

    def to_json(self) -> Dict[str, Any]:
        return {
            "module_id": self.__module_id,
            "drawer_id": self.__drawer_id,
            "pos": self.__pos,
            "size": self.__size,
            "type": self.__type,
            "label": self.__label,
            "is_open": self.__is_open,
        }
