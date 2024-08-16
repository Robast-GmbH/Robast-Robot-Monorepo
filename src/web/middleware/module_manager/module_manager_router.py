from pydantic_models.submodule_process_request import SubmoduleProcessRequest
from pydantic_models.submodule_address import SubmoduleAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_process_manager import ModuleProcessManager

from fastapi import APIRouter, Body


module_manager_router = APIRouter()
module_manager = ModuleManager()
module_process_manager = ModuleProcessManager()


def create_status_response(status: bool):
    return {"status": "success" if status else "failed"}


@module_manager_router.get("", tags=["Modules"])
def get_submodules(robot_name: str):
    return module_manager.get_submodules(robot_name)


@module_manager_router.post("/create_submodule", tags=["Modules"])
def create_submodule(
    submodule_address: SubmoduleAddress,
    position: int = Body(...),
    size: int = Body(...),
    variant: str = Body(...),
):
    has_been_created = module_manager.create_submodule(
        submodule_address.robot_name,
        submodule_address.module_id,
        submodule_address.submodule_id,
        position,
        size,
        variant,
    )
    return create_status_response(has_been_created)


@module_manager_router.post("/delete_submodule", tags=["Modules"])
def delete_submodule(submodule_address: SubmoduleAddress):
    has_been_deleted = module_manager.delete_submodule(submodule_address)
    return create_status_response(has_been_deleted)


@module_manager_router.post("/update_submodule", tags=["Modules"])
def update_submodule(
    submodule_address: SubmoduleAddress,
    variant: str = Body(...),
):
    has_been_updated = module_manager.update_submodule(
        submodule_address,
        variant,
    )
    return create_status_response(has_been_updated)


@module_manager_router.post("/empty_submodule", tags=["Modules"])
def empty_submodule(submodule_address: SubmoduleAddress):
    has_been_emptied = module_manager.empty_submodule(submodule_address)
    return create_status_response(has_been_emptied)


@module_manager_router.post("/update_submodule_content", tags=["Modules"])
def update_submodule_content(
    submodule_address: SubmoduleAddress, items_by_count: dict[str, int] = Body(...)
):
    has_been_updated = module_manager.update_submodule_content(
        submodule_address,
        items_by_count,
    )
    return create_status_response(has_been_updated)


@module_manager_router.post("/free_submodule", tags=["Modules"])
def free_submodule(submodule_address: SubmoduleAddress):
    has_been_freed = module_manager.free_submodule(submodule_address)
    return create_status_response(has_been_freed)


@module_manager_router.post("/reserve_submodule", tags=["Modules"])
def reserve_submodule(
    submodule_address: SubmoduleAddress,
    task_id: str = Body(...),
    user_ids: list[str] = Body(...),
    user_groups: list[str] = Body(...),
):
    has_been_reserved = module_manager.reserve_submodule(
        submodule_address,
        task_id=task_id,
        user_ids=user_ids,
        user_groups=user_groups,
    )
    return create_status_response(has_been_reserved)


@module_manager_router.get("/submodule_process_status", tags=["Modules"])
def read_submodule_process_status(
    robot_name: str,
    module_id: int,
    submodule_id: int,
):
    return {
        "status": module_process_manager.get_submodule_process_status(
            SubmoduleAddress(
                robot_name=robot_name,
                module_id=module_id,
                submodule_id=submodule_id,
            )
        )
    }


@module_manager_router.post("/start_submodule_process", tags=["Modules"])
def start_submodule_process(module_process_data: SubmoduleProcessRequest = Body(...)):
    has_been_started = module_process_manager.start_submodule_process(
        module_process_data
    )
    return create_status_response(has_been_started)


@module_manager_router.post("/open_submodule", tags=["Modules"])
def open_submodule(submodule_address: SubmoduleAddress):
    has_been_opened = module_process_manager.open_submodule(submodule_address)
    return create_status_response(has_been_opened)


@module_manager_router.post("/close_submodule", tags=["Modules"])
def close_submodule(submodule_address: SubmoduleAddress):
    has_been_closed = module_process_manager.close_submodule(submodule_address)
    return create_status_response(has_been_closed)


@module_manager_router.post("/finish_submodule_process", tags=["Modules"])
def finish_submodule_process(submodule_address: SubmoduleAddress):
    has_been_finished = module_process_manager.finish_submodule_process(
        submodule_address
    )
    return create_status_response(has_been_finished)
