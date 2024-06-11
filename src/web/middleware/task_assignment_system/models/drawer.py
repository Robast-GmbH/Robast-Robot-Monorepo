from __future__ import annotations
from typing import Any
from dataclasses import dataclass

@dataclass
class Drawer:
    id: str
    size: int
    is_available: bool = True

    @classmethod
    def from_dict(cls, drawer_dict: dict[str,Any])-> Drawer:
        return cls(id=f"{drawer_dict["module_id"]}_{drawer_dict["drawer_id"]}",
                   size=drawer_dict["size"],
                   )

    def __str__(self) -> str:
        return self.id