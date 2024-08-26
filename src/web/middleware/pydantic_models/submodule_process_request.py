from pydantic import BaseModel
from pydantic_models.submodule_address import SubmoduleAddress


class SubmoduleProcessRequest(BaseModel):
    submodule_address: SubmoduleAddress
    process_name: str
    items_by_change: dict[str, int]
