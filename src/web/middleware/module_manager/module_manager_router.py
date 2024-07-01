from pydantic_models.module_process_data import ModuleProcessData
from pydantic_models.drawer_address import DrawerAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_process_manager import ModuleProcessManager

from fastapi import APIRouter, Body


module_manager_router = APIRouter()
module_manager = ModuleManager()
module_process_manager = ModuleProcessManager()


def create_status_response(status: bool):
    return {"status": "success" if status else "failed"}


@module_manager_router.get("", tags=["Modules"])
def get_modules(robot_name: str):
    return module_manager.get_modules(robot_name)


@module_manager_router.post("/create_module", tags=["Modules"])
def create_module(
    drawer_address: DrawerAddress,
    position: int = Body(...),
    size: int = Body(...),
    variant: str = Body(...),
):
    has_been_created = module_manager.create_module(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
        position,
        size,
        variant,
    )
    return create_status_response(has_been_created)


@module_manager_router.post("/delete_module", tags=["Modules"])
def delete_module(
    drawer_address: DrawerAddress,
):
    has_been_deleted = module_manager.delete_module(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
    )
    return create_status_response(has_been_deleted)


@module_manager_router.post("/empty_module", tags=["Modules"])
def empty_module(
    robot_name: str = Body(...),
    module_id: int = Body(...),
    drawer_id: int = Body(...),
):
    has_been_emptied = module_manager.empty_module(
        robot_name,
        module_id,
        drawer_id,
    )
    return create_status_response(has_been_emptied)


@module_manager_router.post("/update_module_content", tags=["Modules"])
def update_module_content(
    drawer_address: DrawerAddress,
    content: dict[str, int] = Body(...),
):
    has_been_updated = module_manager.update_module_content(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
        content,
    )
    return create_status_response(has_been_updated)


@module_manager_router.post("/free_module", tags=["Modules"])
def free_module(
    drawer_address: DrawerAddress,
):
    has_been_freed = module_manager.free_module(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
    )
    return create_status_response(has_been_freed)


@module_manager_router.post("/reserve_module", tags=["Modules"])
def reserve_module(
    drawer_address: DrawerAddress,
    user_ids: list[str] = Body(...),
    user_groups: list[str] = Body(...),
):
    has_been_reserved = module_manager.reserve_module(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
        user_ids=user_ids,
        user_groups=user_groups,
    )
    return create_status_response(has_been_reserved)


@module_manager_router.get("/module_process_status", tags=["Modules"])
def module_process_status(
    drawer_address: DrawerAddress,
):
    return {
        "status": module_process_manager.get_module_process_status(
            drawer_address.robot_name,
            drawer_address.module_id,
            drawer_address.drawer_id,
        )
    }


@module_manager_router.post("/start_module_process", tags=["Modules"])
def start_module_process(
    robot_name: str = Body(...),
    module_process_data: ModuleProcessData = Body(...),
):
    has_been_started = module_process_manager.start_module_process(
        robot_name,
        module_process_data,
    )
    return create_status_response(has_been_started)


@module_manager_router.post("/open_drawer", tags=["Modules"])
def open_drawer(drawer_address: DrawerAddress):
    has_been_opened = module_process_manager.open_drawer(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
    )
    return create_status_response(has_been_opened)


@module_manager_router.post("/close_drawer", tags=["Modules"])
def close_drawer(drawer_address: DrawerAddress):
    has_been_closed = module_process_manager.close_drawer(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
    )
    return create_status_response(has_been_closed)


@module_manager_router.post("/finish_module_process", tags=["Modules"])
def finish_module_process(drawer_address: DrawerAddress):
    has_been_finished = module_process_manager.finish_module_process(
        drawer_address.robot_name,
        drawer_address.module_id,
        drawer_address.drawer_id,
    )
    return create_status_response(has_been_finished)
