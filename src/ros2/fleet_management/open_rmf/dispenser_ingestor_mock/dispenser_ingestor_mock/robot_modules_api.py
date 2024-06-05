import requests

DRAWER_IDLE = 0
DRAWER_OPEN = 1


class RobotModulesAPI:
    def __init__(self, api_url) -> None:
        self.__api_url = "http://10.10.23.6:8001"
        self.__robot_name = "rb_theron"
        self.__module_id = 0
        self.__drawer_id = 0

    def start_module_process(self,drawer_address,process_type):
        try:
            module_and_drawer_id = drawer_address.split("_")
            self.__module_id = int(module_and_drawer_id[0])
            self.__drawer_id = int(module_and_drawer_id[1])
            self.payload = module_and_drawer_id[2]
            response = requests.post(
                f"{self.__api_url}/start_module_process?module_id={self.__module_id}&drawer_id={self.__drawer_id}&process_name={process_type}&payload={self.payload}"
            )
            response.raise_for_status()
        except Exception as e:
            print(f"Start module process failed: {e}")
        pass

    def update_module_process_status(self):
        try:
            response = requests.get(
                f"{self.__api_url}/module_process_status"
            )
            response.raise_for_status()
            return response.json()["success"]["state"]
        except Exception as e:
            print(f"Request for module process status failed: {e}")
            return None

