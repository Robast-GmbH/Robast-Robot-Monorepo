from task_assignment_system.models.node import Node
import time


class Task:
    def __init__(
        self,
        id,
        task_type,
        requires_task_id,
        drawer,
        target: Node,
    ):
        self.id = id
        self.task_type = task_type
        self.requires_task_id = requires_task_id
        self.drawer = drawer
        self.target = target

    def to_json(self):
        return {
            "type": "robot_task_request",
            "robot": "rb_theron",
            "fleet": "deliveryRobot",
            "request": {
                "unix_millis_earliest_start_time": int(time.time() * 1000),
                "unix_millis_request_time": int(time.time() * 1000),
                "priority": {"value": 0, "type": "binary"},
                "category": "compose",
                "description": {
                    "category": "custom_action",
                    "phases": [
                        {
                            "activity": {
                                "category": "sequence",
                                "description": {
                                    "activities": [
                                        {
                                            "category": self.task_type,
                                            "description": {
                                                "place": self.target.id,
                                                "handler": (
                                                    "drawer_ingestor"
                                                    if self.task_type == "dropoff"
                                                    else "drawer_dispenser"
                                                ),
                                                "payload": {
                                                    "sku": f"{self.drawer.id}_some_stuff",
                                                    "quantity": 1,
                                                },
                                            },
                                        },
                                    ]
                                },
                            }
                        }
                    ],
                },
                "requester": "python_script",
            },
        }
