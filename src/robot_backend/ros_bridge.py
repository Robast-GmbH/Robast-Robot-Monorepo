from roslibpy import Ros
from sub_bridges.robot_pos_bridge import RobotPosBridge
from sub_bridges.nav_bridge import NavBridge
from sub_bridges.module_bridge import ModuleBridge


class RosBridge:
    def __init__(self, ip: str, port: int) -> None:
        self.ros = Ros(ip, port)
        self.ros.run()
        self.robot_pos_bridge = RobotPosBridge(self.ros)
        self.nav_bridge = NavBridge(self.ros)
        self.module_bridge = ModuleBridge(self.ros)
