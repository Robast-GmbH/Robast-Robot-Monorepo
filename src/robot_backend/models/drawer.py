from __future__ import annotations
from thread_safe_dict import ThreadSafeDict
import yaml

TYPE_MANUAL_DRAWER = "manual_drawer"
TYPE_ELECTRIC_DRAWER = "electric_drawer"


class Drawer:
    def __init__(self, module_id, drawer_id, pos, size, type):
        self.module_id = module_id
        self.drawer_id = drawer_id
        self.pos = pos
        self.size = size
        self.type = type
        self.is_open = False

    instances = ThreadSafeDict()
    ids = []

    @classmethod
    def load_drawers(cls, path):
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        for drawer in data["modules"]:
            cls.create_drawer_from_json(drawer)

    @classmethod
    def create_drawer_from_json(cls, json):
        drawer = Drawer(
            json["module_id"],
            json["drawer_id"],
            json["pos"],
            json["size"],
            json["type"],
        )
        id = f"{drawer.module_id}_{drawer.drawer_id}"
        cls.instances.update(id,drawer)
        cls.ids.append(id)
        return drawer

    @classmethod
    def drawers_as_json(cls):
        return [cls.instances.get(id).to_json() for id in cls.ids]

    @classmethod
    def get_drawer(cls, module_id, drawer_id)-> Drawer:
        id = f"{module_id}_{drawer_id}"
        return cls.instances.get(id)

    def to_json(self):
        return {
            "module_id": self.module_id,
            "drawer_id": self.drawer_id,
            "pos": self.pos,
            "size": self.size,
            "type": self.type,
            "is_open": self.is_open,
        }
