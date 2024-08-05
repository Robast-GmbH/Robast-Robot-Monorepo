from fastapi import APIRouter

from task_system.task_assignment_system import TaskAssignmentSystem
from task_system.task_repository import TaskRepository
from pydantic_models.task import Task

task_system_router = APIRouter()
task_repository = TaskRepository()
task_assignment_system = TaskAssignmentSystem()


@task_system_router.post("/task_assignment", tags=["Tasks"])
def post_task_assignment(request: Task):
    success, message = task_assignment_system.receive_task(request)
    return {"success": success, "message": message}


@task_system_router.get("/", tags=["Tasks"])
def get_assigned_tasks():
    tasks = {}
    for robot in task_assignment_system.robots.values():
        tasks[robot.name] = robot.get_robot_tasks()
    return tasks


@task_system_router.get("/by_assignee", tags=["Tasks"])
def get_robot_tasks(robot_name: str):
    return task_assignment_system.robots[robot_name].get_robot_tasks()


@task_system_router.post("/create_task", tags=["Tasks"])
def create_task(task: Task):
    task_repository.create_task(task)
    return task


@task_system_router.get("/task", tags=["Tasks"])
def get_task(task_id: str):
    return task_repository.read_task(task_id)


@task_system_router.get("/tasks", tags=["Tasks"])
def get_tasks():
    return task_repository.read_tasks()


@task_system_router.get("/subtasks", tags=["Tasks"])
def get_subtasks(task_id: str):
    subtasks = task_repository.read_subtasks(task_id)
    return [subtask.to_json() for subtask in subtasks]


@task_system_router.put("/update_task", tags=["Tasks"])
def update_task(task: Task):
    return task_repository.update_task(task)


@task_system_router.delete("/delete_task", tags=["Tasks"])
def delete_task(task_id: str):
    return task_repository.delete_task(task_id)
