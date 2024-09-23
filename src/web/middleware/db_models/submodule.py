from sqlalchemy import Column, Integer, String, JSON
from sqlalchemy.orm import DeclarativeBase
from pydantic_models.submodule_address import SubmoduleAddress
from typing import Any


class Base(DeclarativeBase):
    pass


class Submodule(Base):
    MODULE_TYPE_BIT_MASK = 0x0F
    MODULE_UNIQUE_ID_LENGTH = 16
    MODULE_TYPES = {
        "MANUAL_DRAWER_10x40": 0b00010001,
        "MANUAL_DRAWER_20x40": 0b00010010,
        "MANUAL_DRAWER_30x40": 0b00010011,
        "E_DRAWER_10x40": 0b00010100,
        "PARTIAL_DRAWER_10x40x8": 0b00010101,
        "DINNER_TRAYS": 0b00010110,
        "SURGERY_TOOLS": 0b00010111,
    }
    MODULE_SIZES_BY_TYPES = {
        1: ["MANUAL_DRAWER_10x40", "E_DRAWER_10x40"],
        2: ["MANUAL_DRAWER_20x40"],
        3: ["MANUAL_DRAWER_30x40"],
        5: ["PARTIAL_DRAWER_10x40x8"],
    }

    __tablename__ = "submodules"

    robot_name = Column(String, index=True)
    module_id = Column(Integer, primary_key=True)
    submodule_id = Column(Integer, primary_key=True)
    position = Column(Integer)
    size = Column(Integer)
    variant = Column(String)
    module_process_status = Column(String)
    module_process_type = Column(String)
    module_process_items_by_change = Column(JSON)
    items_by_count = Column(JSON)
    reserved_for_task = Column(String)
    reserved_for_ids = Column(JSON)
    reserved_for_groups = Column(JSON)

    def update_data(self, submodule: "Submodule") -> None:
        self.robot_name = submodule.robot_name
        self.module_id = submodule.module_id
        self.submodule_id = submodule.submodule_id
        self.position = submodule.position
        self.size = submodule.size
        self.variant = submodule.variant
        self.module_process_status = submodule.module_process_status
        self.module_process_type = submodule.module_process_type
        self.module_process_items_by_change = submodule.module_process_items_by_change
        self.items_by_count = submodule.items_by_count
        self.reserved_for_task = submodule.reserved_for_task
        self.reserved_for_ids = submodule.reserved_for_ids
        self.reserved_for_groups = submodule.reserved_for_groups

    def get_address(self) -> SubmoduleAddress:
        return SubmoduleAddress(
            robot_name=self.robot_name,
            module_id=self.module_id,
            submodule_id=self.submodule_id,
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "address": {
                "robot_name": self.robot_name,
                "module_id": self.module_id,
                "submodule_id": self.submodule_id,
            },
            "position": self.position,
            "size": self.size,
            "variant": self.variant,
            "module_process_status": self.module_process_status,
            "module_process_type": self.module_process_type,
            "module_process_items_by_change": self.module_process_items_by_change,
            "items_by_count": self.items_by_count,
            "reserved_for_task": self.reserved_for_task,
            "reserved_for_ids": self.reserved_for_ids,
            "reserved_for_groups": self.reserved_for_groups,
        }

    def is_module_type(self, module_type: int):
        extracted_module_type = self.module_id >> Submodule.MODULE_UNIQUE_ID_LENGTH
        return extracted_module_type == module_type

    def is_size(self, size: int) -> bool:
        module_types = Submodule.MODULE_SIZES_BY_TYPES.get(size, [])
        for module_type in module_types:
            if self.is_module_type(Submodule.MODULE_TYPES[module_type]):
                return True
        return False
