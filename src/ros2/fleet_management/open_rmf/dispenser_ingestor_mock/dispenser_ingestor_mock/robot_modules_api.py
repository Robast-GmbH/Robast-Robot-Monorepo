import requests
from dispenser_ingestor_mock.submodule_process import SubmoduleProcess


class RobotModulesAPI:
    def __init__(self, middleware_url: str) -> None:
        self.__middleware_url = middleware_url

    def start_submodule_process(
        self, submodule_process_request: SubmoduleProcess
    ) -> None:
        try:
            robot_name = submodule_process_request.robot_name
            module_id = submodule_process_request.module_id
            submodule_id = submodule_process_request.submodule_id
            process_name = submodule_process_request.process_name
            items_by_change = submodule_process_request.items_by_change
            data = {
                "drawer_address": {
                    "robot_name": robot_name,
                    "module_id": module_id,
                    "submodule_id": submodule_id,
                },
                "process_name": process_name,
                "items_by_change": items_by_change,
            }
            response = requests.post(
                f"{self.__middleware_url}/modules/start_submodule_process?robot_name={submodule_process_request.robot_name}",
                json=data,
            )
            response.raise_for_status()
        except Exception as e:
            print(f"Start module process failed: {e}")

    def update_submodule_process_status(
        self, robot_name: str, module_id: int, submodule_id: int
    ) -> str | None:
        try:
            response = requests.get(
                f"{self.__middleware_url}/modules/submodule_process_status?robot_name={robot_name}&module_id={module_id}&submodule_id={submodule_id}"
            )
            response.raise_for_status()
            return response.json()["status"]
        except Exception as e:
            print(f"Request for module process status failed: {e}")
            return None
