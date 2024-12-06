from typing import Any, Dict
import uuid


class RobastError:
    def __init__(self, code: str, data: str, description: str) -> None:
        self.id = uuid.uuid4()
        self.code = code
        self.data = data
        self.description = description

    @classmethod
    def from_dict(cls, error_dict: Dict[str, Any]) -> "RobastError":
        return cls(
            error_dict["error_code"],
            error_dict["error_data"],
            error_dict["error_description"],
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "error_code": self.code,
            "error_data": self.data,
            "error_description": self.description,
        }
