from pydantic_models.module_process_data import ModuleProcessData
from module_manager.module_manager import ModuleManager
from module_manager.module_process_manager import ModuleProcessManager

from fastapi import APIRouter


module_manager_router = APIRouter()
module_manager = ModuleManager()
module_process_manager = ModuleProcessManager()


@module_manager_router.get("/", tags=["Modules"])
def get_modules(robot_name: str):
    return module_manager.get_modules(robot_name)


@module_manager_router.post("/create_module", tags=["Modules"])
def create_module(robot_name: str, module_id: int, drawer_id: int, size: int):
    return module_manager.create_module(robot_name, module_id, drawer_id, size)


@module_manager_router.post("/delete_module", tags=["Modules"])
def delete_module(robot_name: str, module_id: int, drawer_id: int):
    return module_manager.delete_module(robot_name, module_id, drawer_id)


@module_manager_router.post("/empty_module", tags=["Modules"])
def empty_module(robot_name: str, module_id: int, drawer_id: int):
    return module_manager.empty_module(robot_name, module_id, drawer_id)


@module_manager_router.post("/update_module_content", tags=["Modules"])
def update_module_content(
    robot_name: str, module_id: int, drawer_id: int, item_id: str, quantity: int
):
    return module_manager.update_module_content(
        robot_name, module_id, drawer_id, item_id, quantity
    )


@module_manager_router.post("/free_module", tags=["Modules"])
def free_module(robot_name: str, module_id: int, drawer_id: int):
    return module_manager.free_module(robot_name, module_id, drawer_id)


@module_manager_router.post("/reserve_module", tags=["Modules"])
def reserve_module(
    robot_name: str,
    module_id: int,
    drawer_id: int,
    user_ids: list[str],
    user_groups: list[str],
):
    return module_manager.reserve_module(
        robot_name, module_id, drawer_id, user_ids=user_ids, user_groups=user_groups
    )


@module_manager_router.get("/module_process_status", tags=["Modules"])
def module_process_status(robot_name: str, module_id: int, drawer_id: int):
    return {
        "status": module_process_manager.get_module_process_status(
            robot_name, module_id, drawer_id
        )
    }


@module_manager_router.post("/start_module_process", tags=["Modules"])
def start_module_process(robot_name: str, module_process_data: ModuleProcessData):
    return module_process_manager.start_module_process(robot_name, module_process_data)


@module_manager_router.post("/open_drawer", tags=["Modules"])
def open_drawer(robot_name: str, module_id: int, drawer_id: int):
    return module_process_manager.open_drawer(robot_name, module_id, drawer_id)


@module_manager_router.post("/close_drawer", tags=["Modules"])
def close_drawer(robot_name: str, module_id: int, drawer_id: int):
    return module_process_manager.close_drawer(robot_name, module_id, drawer_id)


@module_manager_router.post("/finish_module_process", tags=["Modules"])
def finish_module_process(robot_name: str, module_id: int, drawer_id: int):
    return module_process_manager.finish_module_process(
        robot_name, module_id, drawer_id
    )
