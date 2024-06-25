from pydantic import BaseModel


class DrawerAddress(BaseModel):
    robot_name: str
    module_id: int
    drawer_id: int
