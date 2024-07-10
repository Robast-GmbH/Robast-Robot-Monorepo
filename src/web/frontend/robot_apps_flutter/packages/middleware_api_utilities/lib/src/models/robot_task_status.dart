import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class RobotTaskStatus {
  RobotTaskStatus({
    required this.activeTask,
    required this.queuedTasks,
  });

  factory RobotTaskStatus.fromJson(Map<String, dynamic> json) {
    return RobotTaskStatus(
      activeTask: json['active'] == null ? null : Task.fromJson(json['active'] as Map<String, dynamic>),
      queuedTasks: (json['queued'] as List).map((e) => Task.fromJson(e as Map<String, dynamic>)).toList(),
    );
  }

  final Task? activeTask;
  final List<Task> queuedTasks;
}
