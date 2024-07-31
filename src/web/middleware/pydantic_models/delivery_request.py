from pydantic import BaseModel, Field
from typing import Dict, List, Optional


class DeliveryRequest(BaseModel):
    required_drawer_type: int
    items_by_change: Dict[str, int]
    target_id: Optional[str] = None
    start_id: Optional[str] = None
    sender_user_ids: List[str] = Field(default_factory=list)
    sender_user_groups: List[str] = Field(default_factory=list)
    recipient_user_ids: List[str] = Field(default_factory=list)
    recipient_user_groups: List[str] = Field(default_factory=list)
