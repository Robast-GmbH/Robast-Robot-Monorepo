from configs.url_config import ROBOT_NAME_TO_IP, ROBOT_API_PORT
from fastapi import HTTPException


class URLHelper:
    @staticmethod
    def get_robot_url(robot_name: str):
        robot_ip = ROBOT_NAME_TO_IP.get(robot_name, None)
        if robot_ip is None:
            raise HTTPException(404, detail="Robot name not found")
        return f"http://{robot_ip}:{ROBOT_API_PORT}"
