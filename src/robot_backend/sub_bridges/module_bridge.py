from sub_bridges.base_bridge import BaseBridge
from thread_safe_dict import ThreadSafeDict
from roslibpy import Ros
import yaml

class Module:
    def __init__(self, module_id, drawer_id, is_e_drawer):
        self.module_id = module_id
        self.drawer_id = drawer_id
        self.is_e_drawer = is_e_drawer

class ModuleBridge(BaseBridge):
    def __init__(self, ros: Ros) -> None:
        super().__init__(ros)
        self.modules = []
        self.load_config('/workspace/src/robot_backend/configs/module_config.yaml')
        self.start_subscriber('/drawer_is_open', 'communication_interfaces/msg/DrawerStatus',on_msg_callback=self.on_msg_callback)
        self.drawer_tree_publisher = self.start_publisher('/trigger_drawer_tree', 'communication_interfaces/msg/DrawerAddress')
        self.electric_drawer_tree_publisher = self.start_publisher('/trigger_electric_drawer_tree', 'communication_interfaces/msg/DrawerAddress')
        self.close_drawer_publisher = self.start_publisher('/close_drawer', 'communication_interfaces/msg/DrawerAddress')


    def load_config(self, path):
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        for module in data["modules"]:
            module_id = module["module_id"]
            drawer_id = module["drawer_id"]
            is_e_drawer = module["is_e_drawer"]
            self.context.update(f'drawer_is_open_{module_id}_{drawer_id}', False)
            self.modules.append(Module(module_id, drawer_id, is_e_drawer))
        

    def on_msg_callback(self, msg):
        module_id = msg["drawer_address"]["module_id"]
        drawer_id = msg["drawer_address"]["drawer_id"]
        self.context.update(f'drawer_is_open_{module_id}_{drawer_id}', msg["drawer_is_open"])

    def get_drawer_by_id(self, module_id ,drawer_id):
        for module in self.modules:
            if module.module_id == module_id and module.drawer_id == drawer_id:
                return module
        return None

    def open_drawer(self, module_id, drawer_id):
        module = self.get_drawer_by_id(module_id, drawer_id)
        if module is None:
            print("Module not found")
            return False
        if module.is_e_drawer:
            self.electric_drawer_tree_publisher.publish({'module_id': module_id, 'drawer_id': drawer_id})
        else:
            self.drawer_tree_publisher.publish({'module_id': module_id, 'drawer_id': drawer_id})
        return True
    
    def close_drawer(self, module_id, drawer_id):
        module = self.get_drawer_by_id(module_id, drawer_id)
        if module is None:
            print("Module not found")
            return False
        if module.is_e_drawer:
            self.close_drawer_publisher.publish({'module_id': module_id, 'drawer_id': drawer_id})
            return True
        else:
            print("Tried closing a manual drawer.")
            return False

    