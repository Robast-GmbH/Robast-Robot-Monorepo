from pydantic import BaseModel


class SubmoduleStatus(BaseModel):
    is_open: bool
    is_stall_guard_triggered: bool
    opening_timed_out: bool

    @classmethod
    def from_json(cls, json_data: dict[str, bool]) -> "SubmoduleStatus":
        return cls(
            is_open=json_data["is_open"],
            is_stall_guard_triggered=json_data["is_stall_guard_triggered"],
            opening_timed_out=json_data["opening_timed_out"],
        )
