from pydantic import BaseModel
from typing import Optional


class CreateUserRequest(BaseModel):
    title: str
    first_name: str
    last_name: str
    station: str
    room: str
    user_groups: list[str]


class UpdateUserRequest(BaseModel):
    id: str
    title: Optional[str] = None
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    station: Optional[str] = None
    room: Optional[str] = None
    user_groups: Optional[list[str]] = None


class DeleteUserRequest(BaseModel):
    id: str
