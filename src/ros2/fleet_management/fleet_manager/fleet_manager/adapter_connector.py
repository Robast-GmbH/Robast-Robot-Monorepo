# connection interface between the fleet adapter and the fleet manager
from fastapi import Depends, FastAPI
from typing import Optional
from threading import Thread
import rclpy
from rclpy.executors import SingleThreadedExecutor # Or MultiThreadedExecutor if needed
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
import socket
'''
wie könnte man das umsetzen?
option a:
- mqtt verbindung und wenn ein topic empfangen wird, dann lässt man den ros2 subscriber oder service spinnen bis antwort da ist und schickt die dann zurück
- nachteil: gibt keine responses mehr bis ros2 rückmeldung gegeben hat, da das blocking wäre
option b:
- restapi mit fastapi. ich werde hier eh nur angefragt und muss nciht von mir aus iwo hin senden. deswegen kann ich dann beim erhalt eines aufrufs auch wieder spinnen und blocken
- nachteil: ich muss das ganze in einem extra thread laufen lassen, da ich sonst die antworten nicht mehr senden kann

gedanken:
sachen wie eine nav anfrage also action/service calls, wandel ich in einen seperaten thread um und subscriber requests behandel ich blockend und spinne den einfach
'''

app = FastAPI()

class ROS2NodeManager:
    def __init__(self):
        rclpy.init()
        self.node = Node("fastapi_ros2_interface")
        self.subscriber = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        self.executor = SingleThreadedExecutor()  
        self.executor.add_node(self.node)
        self.received_map = None

    def map_callback(self, msg):
        self.received_map = msg

    def start(self):
        self.executor.spin()

    def stop(self):  # Important for clean shutdown
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

def get_ros2_node() -> ROS2NodeManager:
    ros2_node_manager = ROS2NodeManager()
    ros2_node_manager.start()
    return ros2_node_manager

@app.get("/navigate")
def navigate(ros2_node: ROS2NodeManager = Depends(get_ros2_node)):
    print(ros2_node.received_map)
    # Implement your logic here
    pass

@app.post("/start_activity")
def start_activity():
    # Implement your logic here
    pass

@app.post("/stop")
def stop():
    # Implement your logic here
    pass

@app.post("/position")
def get_position():
    # Implement your logic here
    pass

@app.post("/map")
def get_map():
    # Implement your logic here
    pass

@app.post("/battery_soc")
def get_battery_soc():
    # Implement your logic here
    pass

@app.post("/is_command_completed")
def is_command_completed():
    # Implement your logic here
    pass

def find_available_port(start_port: int) -> Optional[int]:
    for port in range(start_port, 65536):  # The maximum port number is 65535
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("0.0.0.0", port))
                return port
            except OSError:
                pass
    return None

def api_main():
    import uvicorn
    port = find_available_port(8000)
    if port is not None:
        uvicorn.run(app, host="0.0.0.0", port=port)
    else:
        print("No available port found.")

if __name__ == "__main__":
    api_main()
