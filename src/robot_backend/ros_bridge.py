from roslibpy import Ros
from sub_bridges.robot_pos_bridge import RobotPosBridge
from sub_bridges.nav_bridge import NavBridge
from sub_bridges.module_bridge import ModuleBridge
from sub_bridges.nfc_bridge import NfcBridge
from sub_bridges.disinfection_module_bridge import DisinfectionModuleBridge
from sub_bridges.error_bridge import ErrorBridge
from sub_bridges.robot_status_bridge import RobotStatusBridge
from sub_bridges.qr_bridge import QrBridge


class RosBridge:
    def __init__(self, ip: str, port: int) -> None:
        self.ros = Ros(ip, port)
        self.ros.run()
        self.robot_pos_bridge = RobotPosBridge(self.ros)
        self.nav_bridge = NavBridge(self.ros)
        self.module_bridge = ModuleBridge(self.ros)
        self.nfc_bridge = NfcBridge(self.ros)
        self.disinfection_module_bridge = DisinfectionModuleBridge(self.ros)
        self.error_bridge = ErrorBridge(self.ros)
        self.robot_status_bridge = RobotStatusBridge(self.ros)
        self.qr_bridge = QrBridge(self.ros)
