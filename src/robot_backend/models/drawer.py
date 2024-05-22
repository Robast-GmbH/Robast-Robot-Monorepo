from __future__ import annotations
from thread_safe_dict import ThreadSafeDict
import yaml

TYPE_MANUAL_DRAWER = "manual_drawer"
TYPE_ELECTRIC_DRAWER = "electric_drawer"


class Drawer:
    def __init__(self, module_id, drawer_id, pos, size, type, label):
        self._module_id = module_id
        self._drawer_id = drawer_id
        self._pos = pos
        self._size = size
        self._type = type
        self._label = label
        self._is_open = False

    instances = ThreadSafeDict()

    @classmethod
    def load_drawers(cls, path):
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        for drawer in data["modules"]:
            cls._create_drawer_from_json(drawer)

    @classmethod
    def drawers_as_json(cls):
        return [drawer._to_json() for drawer in cls.instances.values()]

    @classmethod
    def get_drawer(cls, module_id, drawer_id) -> Drawer | None:
        concatenated_id = f"{module_id}_{drawer_id}"
        try:
            return cls.instances[concatenated_id]
        except KeyError:
            return None

    @classmethod
    def _create_drawer_from_json(cls, json):
        drawer = Drawer(
            json["module_id"],
            json["drawer_id"],
            json["pos"],
            json["size"],
            json["type"],
            json["label"],
        )
        concatenated_id = f"{drawer._module_id}_{drawer._drawer_id}"
        cls.instances[concatenated_id] = drawer

        return drawer

    def set_is_open(self, is_open):
        self._is_open = is_open

    def _to_json(self):
        return {
            "module_id": self._module_id,
            "drawer_id": self._drawer_id,
            "pos": self._pos,
            "size": self._size,
            "type": self._type,
            "label": self._label,
            "is_open": self._is_open,
        }
