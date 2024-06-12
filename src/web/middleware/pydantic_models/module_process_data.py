from pydantic import BaseModel


class ModuleProcessData(BaseModel):
    module_id: int
    drawer_id: int
    process_name: str
    payload: str
