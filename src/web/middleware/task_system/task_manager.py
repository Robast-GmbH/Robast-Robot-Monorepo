from task_system.task_repository import TaskRepository
from pydantic_models.task import Task


class TaskManager:
    def __init__(self):
        self.task_repository = TaskRepository()

    def create_task(self, task) -> int | None:
        return self.task_repository.create_task(task)

    def read_task(self, task_id) -> Task | None:
        return self.task_repository.read_task(task_id)

    def read_all_tasks(self) -> list[Task]:
        return self.task_repository.read_tasks()

    def update_task(self, task) -> bool:
        return self.task_repository.update_task(task)

    def delete_task(self, task_id) -> bool:
        return self.task_repository.delete_task(task_id)

    def get_unassigned_tasks(self) -> list[Task]:
        return self.task_repository.read_unassigned_tasks()

    def finish_task(self, task_id) -> bool:
        task = self.task_repository.read_task(task_id)
        if task:
            task.status = "finished"
            return self.task_repository.update_task(task)
        return False

    def finish_subtask(self, task_id, subtask_id) -> bool:
        task = self.task_repository.read_task(task_id)
        if task:
            for subtask in task.subtasks:
                if subtask.id == subtask_id:
                    subtask.status = "finished"
                    return self.task_repository.update_task(task)
        return False

    def start_subtask(self, task_id, subtask_id) -> bool:
        task = self.task_repository.read_task(task_id)
        if task:
            for subtask in task.subtasks:
                if subtask.id == subtask_id:
                    subtask.status = "in_progress"
                    return self.task_repository.update_task(task)
        return False
