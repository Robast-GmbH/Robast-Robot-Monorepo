from task_system.task_repository import TaskRepository
from pydantic_models.task import Task
from pydantic_models.sub_task import SubTask


class TaskManager:
    def __init__(self):
        self.task_repository = TaskRepository()

    def create_task(self, task) -> int | None:
        return self.task_repository.create_task(task)

    def read_task(self, task_id) -> Task | None:
        return self.task_repository.read_task(task_id)

    def read_tasks_by_assignee(
        self, assignee_name: str, limit: int, offset: int
    ) -> list[Task]:
        return self.task_repository.read_tasks_by_assignee(assignee_name, limit, offset)

    def read_finished_tasks_by_assignee(
        self, assignee_name: str, limit: int, offset: int
    ) -> list[Task]:
        return self.task_repository.read_finished_tasks_by_assignee(
            assignee_name, limit, offset
        )

    def read_all_tasks(self) -> list[Task]:
        return self.task_repository.read_tasks()

    def read_subtask(self, subtask_id: str) -> SubTask | None:
        return self.task_repository.read_subtask(subtask_id)

    def read_subtasks_by_task_id(self, task_id) -> list[SubTask]:
        return self.task_repository.read_subtasks_by_task_id(task_id)

    def read_subtasks_by_subtask_ids(self, subtask_ids: list[str]) -> list[SubTask]:
        return self.task_repository.read_subtasks_by_subtask_ids(subtask_ids)

    def update_task(self, task) -> bool:
        return self.task_repository.update_task(task)

    def update_subtask(self, subtask) -> bool:
        return self.task_repository.update_subtask(subtask)

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

    def start_subtask(self, subtask_id) -> bool:
        subtask = self.task_repository.read_subtask(subtask_id)
        if subtask:
            subtask.status = "active"
            return self.task_repository.update_subtask(subtask)
        return False

    def assign_task(self, task_id, assignee_name) -> bool:
        task = self.task_repository.read_task(task_id)
        if task:
            task.assignee_name = assignee_name
            for subtask in task.subtasks:
                subtask.assignee_name = assignee_name
            task.status = "pending"
            return self.task_repository.update_task(task)
        return False
