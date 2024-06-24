from pydantic import BaseModel
from typing import Optional


class CreateUserRequest(BaseModel):
    first_name: str
    last_name: str
    nfc_id: str
    user_groups: list[str]


class UpdateUserRequest(BaseModel):
    id: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    nfc_id: Optional[str] = None
    user_groups: Optional[list[str]] = None


class DeleteUserRequest(BaseModel):
    id: str
