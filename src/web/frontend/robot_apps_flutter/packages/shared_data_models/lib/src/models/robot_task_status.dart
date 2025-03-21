import 'package:shared_data_models/src/models/subtask.dart';

class RobotTaskStatus {
  RobotTaskStatus({
    required this.activeTask,
    required this.queuedTasks,
  });

  factory RobotTaskStatus.fromJson(Map<String, dynamic> json) {
    return RobotTaskStatus(
      activeTask: json['active'] == null ? null : SubTask.fromJson(json['active'] as Map<String, dynamic>),
      queuedTasks: (json['queue'] as List).map((e) => SubTask.fromJson(e as Map<String, dynamic>)).toList(),
    );
  }

  final SubTask? activeTask;
  final List<SubTask> queuedTasks;
}
