import requests

class RobotAPI:
    def __init__(self) -> None:
        self.api_url = 'http://localhost:8003'
        self.robot_name = 'rb_theron'
        self.module_id = 2
        self.drawer_id = 0

    def open_drawer(self):
        response = requests.post(f'{self.api_url}/open_drawer?robot_name={self.robot_name}&module_id={self.module_id}&drawer_id={self.drawer_id}')
        return response
    
    def is_drawer_open(self):
        response = requests.get(f'{self.api_url}/modules?robot_name={self.robot_name}').json()
        is_open = response[self.module_id-1]['is_open']
        return is_open