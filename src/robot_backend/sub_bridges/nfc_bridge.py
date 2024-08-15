from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros
from threading import Timer


class NfcBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)

    def write_nfc_tag(self, nfc_tag_id: str, timeout_in_s: int) -> dict[str, str]:
        if not self.__validate_uint16(timeout_in_s):
            return {"status": "failure", "message": "timeout_in_s must be a uint16"}
        raw_nfc_tag_id = nfc_tag_id.replace("-", "").lower()
        self.start_service(
            name="/write_nfc",
            service_type="communication_interfaces/srv/WriteNfcTag",
            values={"nfc_tag_id": raw_nfc_tag_id, "timeout_in_s": timeout_in_s},
        )
        return {
            "status": "success" if self.context["/write_nfc"]["success"] else "failure"
        }

    def read_nfc_tag(self, timeout_in_s: int) -> dict[str, str]:
        if not self.__validate_uint16(timeout_in_s):
            return {"status": "failure", "message": "timeout_in_s must be a uint16"}
        self.start_service(
            name="/read_nfc",
            service_type="communication_interfaces/srv/ReadNfcTag",
            values={"timeout_in_s": timeout_in_s},
        ),
        Timer(5, self.__clear_nfc).start()
        return {"nfc_tag": self.get_nfc_tag()}

    def get_nfc_tag(self) -> dict[str, str]:
        try:
            raw_tag = self.context["/read_nfc"]["nfc_tag_id"]
            if raw_tag:
                nfc_tag_id = self.__convert_to_uuid_format(raw_tag)
                return {"data": nfc_tag_id}
            else:
                return {"data": ""}
        except KeyError:
            return {"data": ""}

    def __convert_to_uuid_format(self, hex_string: str) -> str:
        # Lowercase the string
        hex_string = hex_string.lower()

        # Insert hyphens at the appropriate positions
        uuid_string = f"{hex_string[:8]}-{hex_string[8:12]}-{hex_string[12:16]}-{hex_string[16:20]}-{hex_string[20:]}"

        return uuid_string

    def __validate_uint16(self, value: int) -> bool:
        return 0 <= value <= 65535

    def __clear_nfc(self) -> None:
        self.context["/read_nfc"].clear()
