from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros

class NfcBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.start_subscriber(
            "/nfc_tag",
            "std_msgs/msg/String",
        )

    def get_nfc_tag(self) -> dict[str, str]:
        try:
            return self.context["/nfc_tag"]
        except KeyError:
            return {"data":""}
