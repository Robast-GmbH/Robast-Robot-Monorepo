from pydantic import BaseModel
from typing import Optional, List


class UserResponse(BaseModel):
    id: str
    nfc_id: Optional[str]
    mail: Optional[str]
    title: str
    first_name: str
    last_name: str
    station: str
    room: str
    user_groups: List[str]

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    status: str
    user: UserResponse

    class Config:
        from_attributes = True
