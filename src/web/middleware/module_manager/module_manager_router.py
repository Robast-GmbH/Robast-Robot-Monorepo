from pydantic_models.submodule_process_request import SubmoduleProcessRequest
from pydantic_models.submodule_address import SubmoduleAddress
from module_manager.module_manager import ModuleManager
from module_manager.module_process_manager import ModuleProcessManager
from models.logger import Logger

from fastapi import APIRouter, Body


module_manager_router = APIRouter()
module_manager = ModuleManager()
module_process_manager = ModuleProcessManager()
module_manager_logger = Logger("module_manager", "log/module_manager.log")


def create_status_response(status: bool):
    return {"status": "success" if status else "failure"}


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
    module_manager_logger.info(
        f"create_submodule on {submodule_address.robot_name} at position {position} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} -> {'success' if has_been_created else 'failure'}"
    )
    return create_status_response(has_been_created)


@module_manager_router.post("/delete_submodule", tags=["Modules"])
def delete_submodule(submodule_address: SubmoduleAddress):
    has_been_deleted = module_manager.delete_submodule(submodule_address)
    module_manager_logger.info(
        f"delete_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_deleted else 'failure'}"
    )
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
    module_manager_logger.info(
        f"update_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_updated else 'failure'}"
    )
    return create_status_response(has_been_updated)


@module_manager_router.post("/empty_submodule", tags=["Modules"])
def empty_submodule(submodule_address: SubmoduleAddress):
    has_been_emptied = module_manager.empty_submodule(submodule_address)
    module_manager_logger.info(
        f"empty_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_emptied else 'failure'}"
    )
    return create_status_response(has_been_emptied)


@module_manager_router.post("/update_submodule_content", tags=["Modules"])
def update_submodule_content(
    submodule_address: SubmoduleAddress, items_by_count: dict[str, int] = Body(...)
):
    has_been_updated = module_manager.update_submodule_content(
        submodule_address,
        items_by_count,
    )
    module_manager_logger.info(
        f"update_submodule_content on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
to {items_by_count} \
-> {'success' if has_been_updated else 'failure'}"
    )
    return create_status_response(has_been_updated)


@module_manager_router.post("/free_submodule", tags=["Modules"])
def free_submodule(submodule_address: SubmoduleAddress):
    has_been_freed = module_manager.free_submodule(submodule_address)
    module_manager_logger.info(
        f"free_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_freed else 'failure'}"
    )
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
    module_manager_logger.info(
        f"reserve_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
for task {task_id} and user_ids {user_ids} and user_groups {user_groups} \
-> {'success' if has_been_reserved else 'failure'}"
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
    module_manager_logger.info(
        f"start_submodule_process on {module_process_data.submodule_address.robot_name} \
with module_id {module_process_data.submodule_address.module_id}, submodule_id {module_process_data.submodule_address.submodule_id} \
and items_by_change {module_process_data.items_by_change} \
-> {'success' if has_been_started else 'failure'}"
    )
    return create_status_response(has_been_started)


@module_manager_router.post("/open_submodule", tags=["Modules"])
def open_submodule(submodule_address: SubmoduleAddress):
    has_been_opened = module_process_manager.open_submodule(submodule_address)
    module_manager_logger.info(
        f"open_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_opened else 'failure'}"
    )
    return create_status_response(has_been_opened)


@module_manager_router.post("/close_submodule", tags=["Modules"])
def close_submodule(submodule_address: SubmoduleAddress):
    has_been_closed = module_process_manager.close_submodule(submodule_address)
    module_manager_logger.info(
        f"close_submodule on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_closed else 'failure'}"
    )
    return create_status_response(has_been_closed)


@module_manager_router.post("/finish_submodule_process", tags=["Modules"])
def finish_submodule_process(submodule_address: SubmoduleAddress):
    has_been_finished = module_process_manager.finish_submodule_process(
        submodule_address
    )
    module_manager_logger.info(
        f"finish_submodule_process on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_finished else 'failure'}"
    )
    return create_status_response(has_been_finished)


@module_manager_router.post("/cancel_submodule_process", tags=["Modules"])
def cancel_submodule_process(submodule_address: SubmoduleAddress):
    has_been_cancelled = module_process_manager.cancel_submodule_process(
        submodule_address
    )
    module_manager_logger.info(
        f"cancel_submodule_process on {submodule_address.robot_name} \
with module_id {submodule_address.module_id} and submodule_id {submodule_address.submodule_id} \
-> {'success' if has_been_cancelled else 'failure'}"
    )
    return create_status_response(has_been_cancelled)
