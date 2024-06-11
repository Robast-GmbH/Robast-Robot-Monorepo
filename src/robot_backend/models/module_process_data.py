from pydantic import BaseModel, field_validator


class ModuleProcessData(BaseModel):
    module_id: int
    drawer_id: int
    process_name: str
    payload: str
