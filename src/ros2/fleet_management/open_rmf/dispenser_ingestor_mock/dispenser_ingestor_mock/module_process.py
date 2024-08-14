from __future__ import annotations
import json
from rmf_dispenser_msgs.msg import DispenserRequest
from rmf_ingestor_msgs.msg import IngestorRequest
from dataclasses import dataclass

PICK_UP_PROCESS = "pickup"
DROP_OFF_PROCESS = "dropoff"


@dataclass
class ModuleProcess:
    robot_name: str
    module_id: int
    drawer_id: int
    process_name: str
    items_by_change: dict[str, int]
    request_guid: str
    target_guid: str
    is_in_process: bool

    @classmethod
    def from_request_msg(
        cls, request_msg: DispenserRequest | IngestorRequest
    ) -> ModuleProcess:
        input_str = request_msg.items[0].type_guid
        action_data = json.loads(input_str)

        process_name = (
            PICK_UP_PROCESS
            if isinstance(request_msg, DispenserRequest)
            else DROP_OFF_PROCESS
        )
        return cls(
            robot_name=action_data["parameters"]["drawer_address"]["robot_name"],
            module_id=int(action_data["parameters"]["drawer_address"]["module_id"]),
            drawer_id=int(action_data["parameters"]["drawer_address"]["drawer_id"]),
            process_name=process_name,
            items_by_change=action_data["parameters"]["items_by_change"],
            request_guid=request_msg.request_guid,
            target_guid=request_msg.target_guid,
            is_in_process=True,
        )
