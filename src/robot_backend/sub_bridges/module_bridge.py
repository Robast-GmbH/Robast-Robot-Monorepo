from sub_bridges.base_bridge import BaseBridge
from models.drawer import Drawer, TYPE_ELECTRIC_DRAWER, TYPE_MANUAL_DRAWER
from roslibpy import Ros


class ModuleBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        Drawer.load_drawers("/workspace/src/robot_backend/configs/module_config.yaml")
        self.start_subscriber(
            "/bt_drawer_open",
            "communication_interfaces/msg/DrawerStatus",
            on_msg_callback=self.on_drawer_is_open_msg_callback,
        )
        self.drawer_tree_publisher = self.start_publisher(
            "/trigger_drawer_tree", "communication_interfaces/msg/DrawerAddress"
        )
        self.electric_drawer_tree_publisher = self.start_publisher(
            "/trigger_electric_drawer_tree",
            "communication_interfaces/msg/DrawerAddress",
        )
        self.close_drawer_publisher = self.start_publisher(
            "/close_drawer", "communication_interfaces/msg/DrawerAddress"
        )

        self.current_module_process = {}

    def on_drawer_is_open_msg_callback(self, msg):
        module_id = msg["drawer_address"]["module_id"]
        drawer_id = msg["drawer_address"]["drawer_id"]
        is_open = msg["drawer_is_open"]
        concatenated_id = f"{module_id}_{drawer_id}"
        Drawer.instances[concatenated_id].set_is_open(is_open)

        self.current_module_process["state"] = "open" if is_open else "closed"

    def open_drawer(self, module_id, drawer_id):
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            print("Module not found")
            return False

        self.current_module_process["state"] = "opening"

        if drawer._type == TYPE_ELECTRIC_DRAWER:
            self.electric_drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
        else:
            self.drawer_tree_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
        return True

    def close_drawer(self, module_id, drawer_id):
        drawer = Drawer.get_drawer(module_id, drawer_id)
        if drawer is None:
            print("Module not found")
            return False
        if drawer._type == TYPE_ELECTRIC_DRAWER:
            self.close_drawer_publisher.publish(
                {"module_id": module_id, "drawer_id": drawer_id}
            )
            self.current_module_process["state"] = "closing"
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    def start_module_process(self, module_id, drawer_id, process_name):
        current_module_process_state = self.current_module_process.get("state")
        if (
            current_module_process_state is None
            or current_module_process_state == "finished"
        ):
            self.current_module_process = {
                "module_id": module_id,
                "drawer_id": drawer_id,
                "process_name": process_name,
                "state": "waiting_for_opening_command",
            }

    def finish_module_process(self):
        if self.current_module_process["state"] != "closed":
            return f"Module process has to be in closed state to be finished."
        else:
            self.current_module_process["state"] = "finished"
            return "Module process finished."

    def get_modules(self):
        return Drawer.drawers_as_json()
