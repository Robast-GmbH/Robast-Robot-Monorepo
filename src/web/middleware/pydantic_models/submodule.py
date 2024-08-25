from pydantic import BaseModel
from typing import Dict, Any
from pydantic_models.submodule_address import SubmoduleAddress


class Submodule(BaseModel):
    address: SubmoduleAddress
    position: int
    size: int
    variant: str
    module_process_status: str
    module_process_type: str
    module_process_items_by_change: Dict[str, int]
    items_by_count: Dict[str, int]
    reserved_for_task: str
    reserved_for_ids: list[str]
    reserved_for_groups: list[str]

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "Submodule":
        return cls(
            address=SubmoduleAddress.from_json(json_data["address"]),
            size=json_data["size"],
            position=json_data["position"],
            variant=json_data["variant"],
            module_process_status=json_data["module_process_status"],
            module_process_type=json_data["module_process_type"],
            module_process_items_by_change=json_data["module_process_items_by_change"],
            items_by_count=json_data["items_by_count"],
            reserved_for_task=json_data["reserved_for_task"],
            reserved_for_ids=json_data["reserved_for_ids"],
            reserved_for_groups=json_data["reserved_for_groups"],
        )

    def to_json(self) -> dict[str, Any]:
        return self.model_dump()
