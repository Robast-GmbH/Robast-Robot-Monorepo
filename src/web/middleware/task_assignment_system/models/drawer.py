class Drawer:
    def __init__(self, size: int, id: str):
        self.size = size
        self.id = id
        self.is_available = True

    @classmethod
    def from_dict(cls, drawer_dict):
        return cls(drawer_dict["size"], f"{drawer_dict["module_id"]}_{drawer_dict["drawer_id"]}")

    def __str__(self) -> str:
        return self.id