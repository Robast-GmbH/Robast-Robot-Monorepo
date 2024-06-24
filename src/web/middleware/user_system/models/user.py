from pydantic import BaseModel


class User(BaseModel):
    id: str
    first_name: str
    last_name: str
    nfc_id: str
    user_groups: list[str]
