import requests

DRAWER_IDLE = 0
DRAWER_OPEN = 1


class RobotModulesAPI:
    def __init__(self, middleware_url, robot_name) -> None:
        self.__middleware_url = middleware_url
        self.__robot_name = robot_name
        self.__module_id = 0
        self.__drawer_id = 0

    def start_module_process(self, drawer_address, process_type):
        try:
            module_and_drawer_id = drawer_address.split("_")
            self.__module_id = int(module_and_drawer_id[0])
            self.__drawer_id = int(module_and_drawer_id[1])
            payload = module_and_drawer_id[2]
            data = {
                "module_id": self.__module_id,
                "drawer_id": self.__drawer_id,
                "process_name": process_type,
                "payload": payload,
            }
            response = requests.post(
                f"{self.__middleware_url}/start_module_process?robot_name={self.__robot_name}",
                json=data,
            )
            response.raise_for_status()
        except Exception as e:
            print(f"Start module process failed: {e}")

    def update_module_process_status(self):
        try:
            response = requests.get(
                f"{self.__middleware_url}/module_process_status?robot_name={self.__robot_name}"
            )
            response.raise_for_status()
            return response.json()["success"]["state"]
        except Exception as e:
            print(f"Request for module process status failed: {e}")
            return None
