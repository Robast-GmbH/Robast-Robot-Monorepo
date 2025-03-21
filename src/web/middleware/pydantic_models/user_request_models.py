from pydantic import BaseModel
from typing import Optional


class CreateUserRequest(BaseModel):
    mail: str | None = None
    nfc_id: str | None = None
    title: str
    first_name: str
    last_name: str
    station: str
    room: str
    user_groups: list[str]


class UpdateUserRequest(BaseModel):
    id: str
    nfc_id: Optional[str] = None
    mail: Optional[str] = None
    title: Optional[str] = None
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    station: Optional[str] = None
    room: Optional[str] = None
    user_groups: Optional[list[str]] = None


class DeleteUserRequest(BaseModel):
    id: str


class LoginRequest(BaseModel):
    mail: str
    password: str
