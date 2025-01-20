from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from threading import Timer


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
        self.__read_nfc_service.start_request(values={"timeout_in_s": timeout_in_s}),
        Timer(5, self.__clear_nfc).start()
        return {"status": "success", "nfc_tag": self.get_nfc_tag()}

    def get_nfc_tag(self) -> dict[str, str]:
        try:
            raw_tag = self.context["/read_nfc"]["nfc_tag_id"]
            if raw_tag:
                return {"data": raw_tag}
            else:
                return {"data": ""}
        except KeyError:
            return {"data": ""}

    def __validate_uint16(self, value: int) -> bool:
        return 0 <= value <= 65535

    def __clear_nfc(self) -> None:
        self.context["/read_nfc"].clear()
