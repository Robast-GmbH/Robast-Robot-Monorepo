import requests
from dispenser_ingestor_mock.module_process import ModuleProcess

DRAWER_IDLE = 0
DRAWER_OPEN = 1


class RobotModulesAPI:
    def __init__(self, middleware_url: str) -> None:
        self.__middleware_url = middleware_url

    def start_module_process(
        self,
        module_process_request: ModuleProcess,
    ) -> None:
        try:

            module_id = module_process_request.module_id
            drawer_id = module_process_request.drawer_id
            process_name = module_process_request.process_name
            payload = module_process_request.payload
            data = {
                "module_id": module_id,
                "drawer_id": drawer_id,
                "process_name": process_name,
                "payload": payload,
            }
            response = requests.post(
                f"{self.__middleware_url}/start_module_process?robot_name={module_process_request.robot_name}",
                json=data,
            )
            response.raise_for_status()
        except Exception as e:
            print(f"Start module process failed: {e}")

    def update_module_process_status(self, robot_name: str) -> str | None:
        try:
            response = requests.get(
                f"{self.__middleware_url}/module_process_status?robot_name={robot_name}"
            )
            response.raise_for_status()
            return response.json()["success"]["state"]
        except Exception as e:
            print(f"Request for module process status failed: {e}")
            return None
