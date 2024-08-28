import 'package:middleware_api_utilities/src/models/robot_task_status.dart';
import 'package:middleware_api_utilities/src/models/task.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class TasksApi {
  TasksApi({required this.prefix});
  final String prefix;

  Future<RobotTaskStatus?> getRobotTasks({required String robotName}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/tasks/by_assignee?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToList(response: response);
      Map<String, dynamic> json = {'active': null, 'queued': []};
      for (final task in data) {
        for (final subtask in task['subtasks']) {
          if (subtask['status'] == 'active') {
            json['active'] = subtask;
          } else if (subtask['status'] == 'pending') {
            json['queued'].add(subtask);
          }
        }
      }
      return RobotTaskStatus.fromJson(json);
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
