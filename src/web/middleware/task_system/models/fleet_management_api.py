from typing import Any
import requests


class FleetManagementAPI:
    def __init__(self, url) -> None:
        self.__url = url

    def request_task_status(self, task_id: str) -> dict[str, Any] | None:
        try:
            response = requests.get(f"{self.__url}/tasks/{task_id}/state")
            response.raise_for_status()
        except requests.exceptions.HTTPError:
            return None
        data = response.json()
        return data

    def dispatch_robot_task(self, task: dict[str, Any]) -> dict[str, Any] | None:
        try:
            response = requests.post(f"{self.__url}/tasks/robot_task", json=task)
            response.raise_for_status()
        except requests.exceptions.HTTPError:
            return None
        return response.json()
