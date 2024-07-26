from pydantic import BaseModel
from pydantic_models.drawer_address import DrawerAddress


class ModuleProcessRequest(BaseModel):
    drawer_address: DrawerAddress
    process_name: str
    items_by_change: dict[str, int]
