from typing import Any, Dict
import uuid
from models.deserializer import Deserializer
import sys

sys.path.append("/robast/humble/error_utils/bin")
from error_codes import error_codes_by_interface


class RobastError:
    def __init__(self, code: int, data: str, description: str) -> None:
        self.id = uuid.uuid4()
        self.code = code
        self.description = description

        error_interface = error_codes_by_interface.get(code)
        if error_interface == "communication_interfaces::msg::DrawerAddress":
            self.data = Deserializer.deserialize_drawer_address(bytes(data, "utf-8"))
        elif error_interface == "std::string":
            self.data = data
        else:
            self.data = f"Unknown error code: {data}"

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
