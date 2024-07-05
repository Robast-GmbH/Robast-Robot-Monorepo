from pydantic import BaseModel


class User(BaseModel):
    id: str
    title: str
    first_name: str
    last_name: str
    station: str
    room: str
    user_groups: list[str]
