from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros


class NfcBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.__read_nfc_service = self.create_service(
            "/read_nfc",
            "communication_interfaces/srv/ReadNfcTag",
        )

    def read_nfc_tag(self, timeout_in_s: int) -> dict[str, str]:
        if not self.__validate_uint16(timeout_in_s):
            return {"status": "failure", "message": "timeout_in_s must be a uint16"}
        response = self.__read_nfc_service.start_request(
            values={"timeout_in_s": timeout_in_s}
        )

        if response is None:
            return {"status": "failure", "message": "No response from service"}
        else:
            return {"status": "success", "nfc_tag": response["nfc_tag_id"]}

    def __validate_uint16(self, value: int) -> bool:
        return 0 <= value <= 65535
