from __future__ import annotations
from thread_safe_dict import ThreadSafeDict
import yaml

TYPE_MANUAL_DRAWER = "manual_drawer"
TYPE_ELECTRIC_DRAWER = "electric_drawer"


class Drawer:
    def __init__(self, module_id, drawer_id, pos, size, type, label):
        self.__module_id = module_id
        self.__drawer_id = drawer_id
        self.__pos = pos
        self.__size = size
        self.__type = type
        self.__label = label
        self.__is_open = False

    __instances = ThreadSafeDict()

    @classmethod
    def load_drawers(cls, path):
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        for drawer in data["modules"]:
            cls.create_drawer_from_json(drawer)

    @classmethod
    def drawers_as_json(cls):
        return [drawer.to_json() for drawer in cls.__instances.values()]

    @classmethod
    def get_drawer(cls, module_id, drawer_id) -> Drawer | None:
        concatenated_id = f"{module_id}_{drawer_id}"
        try:
            return cls.__instances[concatenated_id]
        except KeyError:
            return None

    @classmethod
    def create_drawer_from_json(cls, json):
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

    def set_is_open(self, is_open):
        self.__is_open = is_open

    def get_type(self):
        return self.__type

    def to_json(self):
        return {
            "module_id": self.__module_id,
            "drawer_id": self.__drawer_id,
            "pos": self.__pos,
            "size": self.__size,
            "type": self.__type,
            "label": self.__label,
            "is_open": self.__is_open,
        }
