from __future__ import annotations
from typing import Dict, List, Any
from thread_safe_dict import ThreadSafeDict
import yaml

TYPE_MANUAL_DRAWER = "manual_drawer"
TYPE_ELECTRIC_DRAWER = "electric_drawer"


class Submodule:
    def __init__(
        self,
        module_id: int,
        submodule_id: int,
        pos: int,
        size: int,
        type: str,
        label: str,
    ) -> None:
        self.__module_id = module_id
        self.__submodule_id = submodule_id
        self.__pos = pos
        self.__size = size
        self.__type = type
        self.__label = label
        self.__is_open = False

    __instances: ThreadSafeDict[str, Submodule] = ThreadSafeDict()

    @classmethod
    def load_submodules(cls, path: str) -> None:
        with open(path, "r", encoding="UTF-8") as file:
            data = yaml.safe_load(file)
        for submodule in data["modules"]:
            cls.create_submodule_from_json(submodule)

    @classmethod
    def submodules_as_json(cls) -> List[Dict[str, Any]]:
        return [submodule.to_json() for submodule in cls.__instances.values()]

    @classmethod
    def get_submodule(cls, module_id: int, submodule_id: int) -> Submodule | None:
        concatenated_id = f"{module_id}_{submodule_id}"
        try:
            return cls.__instances[concatenated_id]
        except KeyError:
            return None

    @classmethod
    def create_submodule_from_json(cls, json: Dict[str, Any]) -> Submodule:
        submodule = Submodule(
            json["module_id"],
            json["submodule_id"],
            json["pos"],
            json["size"],
            json["type"],
            json["label"],
        )
        concatenated_id = f"{submodule.__module_id}_{submodule.__submodule_id}"
        cls.__instances[concatenated_id] = submodule

        return submodule

    def set_is_open(self, is_open: bool) -> None:
        self.__is_open = is_open

    def get_is_open(self) -> bool:
        return self.__is_open

    def get_type(self) -> str:
        return self.__type

    def to_json(self) -> Dict[str, Any]:
        return {
            "module_id": self.__module_id,
            "submodule_id": self.__submodule_id,
            "pos": self.__pos,
            "size": self.__size,
            "type": self.__type,
            "label": self.__label,
            "is_open": self.__is_open,
        }
