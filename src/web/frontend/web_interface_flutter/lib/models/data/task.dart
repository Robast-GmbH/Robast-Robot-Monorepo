import 'package:web_interface_flutter/models/data/actions/robot_action.dart';

class Task {
  final String robotName;
  final String fleetName;
  final String id;

  final List<RobotAction> actions = [];

  Task({
    required this.id,
    required this.robotName,
    required this.fleetName,
  });

  static Task fromJson(Map<String, dynamic> data) {
    final newTask = Task(
      robotName: data["robot"]["robot_name"],
      fleetName: data["robot"]["fleet_name"],
      id: data["task_id"],
    );

    if (data.containsKey("actions")) {
      newTask.actions.addAll(
        (data["actions"] as List<dynamic>).map((e) => RobotAction.fromJson(e)).toList(),
      );
    }
    newTask.actions.sort((a, b) => a.step.compareTo(b.step));
    return newTask;
  }
}
