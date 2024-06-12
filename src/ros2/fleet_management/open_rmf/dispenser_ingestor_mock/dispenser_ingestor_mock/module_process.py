from __future__ import annotations
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
    payload: str
    request_guid: str
    target_guid: str
    is_in_process: bool

    @classmethod
    def from_request_msg(
        cls, request_msg: DispenserRequest | IngestorRequest
    ) -> ModuleProcess:
        module_id, drawer_id, payload, robot_name = request_msg.items[
            0
        ].type_guid.split(",")
        process_name = (
            PICK_UP_PROCESS
            if isinstance(request_msg, DispenserRequest)
            else DROP_OFF_PROCESS
        )
        return cls(
            robot_name=robot_name,
            module_id=int(module_id),
            drawer_id=int(drawer_id),
            process_name=process_name,
            payload=payload,
            request_guid=request_msg.request_guid,
            target_guid=request_msg.target_guid,
            is_in_process=True,
        )

    def to_json(self):
        return {
            "module_id": self.module_id,
            "drawer_id": self.drawer_id,
            "process_name": self.process_name,
            "payload": self.payload,
        }
