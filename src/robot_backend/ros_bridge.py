from roslibpy import Ros
from thread_safe_dict import ThreadSafeDict
from sub_bridges.robot_pos_bridge import RobotPosBridge
from sub_bridges.nav_bridge import NavBridge


class RosBridge:
    def __init__(self, ip: str, port: int) -> None:
        self.context = ThreadSafeDict()
        self.ros = Ros(ip, port)
        self.ros.run()
        self.robot_pos_bridge = RobotPosBridge(self.ros, self.context)
        self.nav_bridge = NavBridge(self.ros, self.context)
