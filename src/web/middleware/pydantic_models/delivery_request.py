from pydantic import BaseModel, Field
from typing import Dict, List, Optional


class DeliveryRequest(BaseModel):
    required_drawer_type: int
    payload: Dict[str, int]
    target_id: Optional[str] = None
    start_id: Optional[str] = None
    user_ids: List[str] = Field(default_factory=list)
    user_groups: List[str] = Field(default_factory=list)
