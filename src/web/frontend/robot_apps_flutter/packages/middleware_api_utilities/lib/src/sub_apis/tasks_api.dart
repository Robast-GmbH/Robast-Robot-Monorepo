import 'package:middleware_api_utilities/src/models/robot_task_status.dart';
import 'package:middleware_api_utilities/src/models/task.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class TasksApi {
  TasksApi({required this.prefix});
  final String prefix;

  Future<RobotTaskStatus?> getRobotTasks({required String robotName}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/tasks/robot_tasks?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToMap(response: response);

      return RobotTaskStatus.fromJson(data['tasks']);
    } else {
      return null;
    }
  }

  Future<List<Task>?> getFinishedTasks({required String robotName, required int limit, required int offset}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/tasks/finished_by_assignee?robot_name=$robotName&limit=$limit&offset=$offset'));
    if (response != null) {
      final data = RequestService.responseToList(response: response);
      final tasks = <Task>[];
      for (final taskData in data) {
        final task = Task.fromJson(taskData as Map<String, dynamic>);
        tasks.add(task);
      }
      return tasks;
    } else {
      return null;
    }
  }

  Future<bool> postTaskRequest({required Task task}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/tasks/task_assignment'),
      data: task.toJson(),
    );
    return RequestService.wasRequestSuccessful(response: response);
  }
}
