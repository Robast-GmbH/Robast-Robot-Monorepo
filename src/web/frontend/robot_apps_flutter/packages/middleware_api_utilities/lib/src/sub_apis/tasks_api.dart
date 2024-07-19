import 'package:middleware_api_utilities/src/models/robot_task_status.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class TasksApi {
  TasksApi({required this.prefix});
  final String prefix;

  Future<RobotTaskStatus?> getRobotTasks({required String robotName}) async {
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/robot_tasks?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      return RobotTaskStatus.fromJson(data);
    } else {
      return null;
    }
  }

  Future<bool> postTaskRequest({
    required int requiredDrawerType,
    required Map<String, int> payload,
    required List<String> senderAuthUsers,
    required List<String> senderAuthUserGroups,
    required List<String> recipientAuthUsers,
    required List<String> recipientAuthUserGroups,
    String? targetID,
    String? startID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/task_assignment'),
      data: {
        'required_drawer_type': requiredDrawerType,
        'payload': payload,
        'target_id': targetID,
        'start_id': startID,
        'sender_user_ids': senderAuthUsers,
        'sender_user_groups': senderAuthUserGroups,
        'recipient_user_ids': recipientAuthUsers,
        'recipient_user_groups': recipientAuthUserGroups,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }
}
