from fastapi import APIRouter

from pydantic_models.task_request import TaskRequest
from task_system.task_assignment_system import TaskAssignmentSystem
from task_system.task_manager import TaskManager
from pydantic_models.task_request import TaskRequest

task_system_router = APIRouter()
task_manager = TaskManager()
task_assignment_system = TaskAssignmentSystem()


@task_system_router.post("/task_assignment", tags=["Tasks"])
def post_task_assignment(request: TaskRequest):
    success, message = task_assignment_system.receive_task(request)
    return {"status": "success" if success else "failure", "message": message}


@task_system_router.get("/by_assignee", tags=["Tasks"])
def get_tasks_by_assignee(robot_name: str, limit: int = 100, offset: int = 0):
    robot_tasks = task_manager.read_tasks_by_assignee(robot_name, limit, offset)
    return [task.to_json() for task in robot_tasks]


@task_system_router.get("/finished_by_assignee", tags=["Tasks"])
def get_finished_tasks_by_assignee(robot_name: str, limit: int = 100, offset: int = 0):
    robot_tasks = task_manager.read_finished_tasks_by_assignee(
        robot_name, limit, offset
    )
    return [task.to_json() for task in robot_tasks]


@task_system_router.get("/robot_tasks", tags=["Tasks"])
def get_robot_tasks(robot_name: str):
    robot_tasks = task_assignment_system.get_robot_tasks(robot_name)
    return robot_tasks


@task_system_router.post("/create_task", tags=["Tasks"])
def create_task(task: TaskRequest):
    task_manager.create_task(task)
    return task


@task_system_router.get("/task", tags=["Tasks"])
def get_task(task_id: str):
    return task_manager.read_task(task_id)


@task_system_router.get("/tasks", tags=["Tasks"])
def get_tasks():
    tasks = task_manager.read_all_tasks()
    return [task.to_json() for task in tasks]


@task_system_router.get("/subtasks", tags=["Tasks"])
def get_subtasks(task_id: str):
    subtasks = task_manager.read_subtasks_by_task_id(task_id)
    return [subtask.to_json() for subtask in subtasks]


@task_system_router.put("/update_task", tags=["Tasks"])
def update_task(task: TaskRequest):
    return task_manager.update_task(task)


@task_system_router.delete("/delete_task", tags=["Tasks"])
def delete_task(task_id: str):
    return task_manager.delete_task(task_id)


@task_system_router.post("/robot_is_task_queue_open", tags=["Tasks"])
def set_robot_is_task_queue_open(robot_name: str, is_open: bool):
    return task_assignment_system.set_robot_is_task_queue_open(robot_name, is_open)


@task_system_router.get("/robot_is_task_queue_open", tags=["Tasks"])
def get_robot_is_task_queue_open(robot_name: str):
    return task_assignment_system.get_robot_is_task_queue_open(robot_name)
