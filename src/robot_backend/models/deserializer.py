import struct
from typing import Tuple, Dict, Any


class Deserializer:
    DRAWER_ADDRESS_FORMAT = "IB"

    @staticmethod
    def deserialize_message(serialized_data: bytes, format: str) -> Tuple[Any, ...]:
        expected_size = struct.calcsize(format)
        if len(serialized_data) < expected_size:
            raise ValueError(
                f"Invalid data size: expected at least {expected_size}, got {len(serialized_data)}"
            )

        return struct.unpack(format, serialized_data[:expected_size])

    @staticmethod
    def deserialize_drawer_address(serialized_data: bytes) -> Dict[str, int]:
        module_id, submodule_id = Deserializer.deserialize_message(
            serialized_data, Deserializer.DRAWER_ADDRESS_FORMAT
        )
        return {"module_id": module_id, "submodule_id": submodule_id}
