from base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros

class DrawerBridge(BaseBridge):
    def __init__(self, ros: Ros, context: ThreadSafeDict) -> None:
        super().__init__(ros,context)