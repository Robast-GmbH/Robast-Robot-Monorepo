from sub_bridges.base_bridge import BaseBridge
from roslibpy import Ros


class QrBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.__read_qr_code_service = self.create_service(
            "/read_qr_code",
            "communication_interfaces/srv/ReadQrCode",
        )

    def read_qr_code(self, expected_data: str, timeout_in_s: int) -> dict[str, str]:
        if not self.__validate_uint16(timeout_in_s):
            return {"status": "failure", "message": "timeout_in_s must be a uint16"}
        response = self.__read_qr_code_service.start_request(
            values={"expected_data": expected_data, "timeout_in_s": timeout_in_s}
        )

        if response is None:
            return {"status": "failure", "message": "No response from service"}
        else:
            return {"status": "success" if response["success"] else "failure"}

    def __validate_uint16(self, value: int) -> bool:
        return 0 <= value <= 65535
