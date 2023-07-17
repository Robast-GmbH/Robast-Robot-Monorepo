from pydantic import BaseModel


class Location(BaseModel):
    sec: int
    nanosec: int
    x: float
    y: float
    yaw: float
    level_name: str
    