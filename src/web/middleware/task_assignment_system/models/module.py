class Module:
    def __init__(self, size: int, id: str):
        self.size = size
        self.id = id
        self.is_available = True

    @classmethod
    def from_dict(cls, module_dict):
        return cls(module_dict["size"], f"{module_dict["module_id"]}_{module_dict["drawer_id"]}")

    @classmethod
    def module_setup_from_json(cls, json):
        return [cls.from_dict(drawer_dict) for drawer_dict in json]
