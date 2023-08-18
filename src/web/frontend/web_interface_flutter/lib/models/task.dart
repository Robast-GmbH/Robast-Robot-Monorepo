import 'package:web_interface_flutter/models/actions/robot_action.dart';

class Task {
  final String robotName;
  final String fleetName;
  final int ownerID;
  final int id;

  final List<RobotAction> actions = [];

  Task({
    required this.id,
    required this.ownerID,
    required this.robotName,
    required this.fleetName,
  });

  static Task fromJson(Map<String, dynamic> data) {
    final newTask = Task(
      robotName: data["robot_name"],
      fleetName: data["fleet_name"],
      id: data["id"],
      ownerID: data["owner_id"],
    );

    if (data.containsKey("actions")) {
      (data["actions"] as List<Map<String, dynamic>>).sort(
        (a, b) => a["step"] < b["step"],
      );
      newTask.actions.addAll(
        (data["actions"] as List<Map<String, dynamic>>).map((e) => RobotAction.fromJson(e["action"])).toList(),
      );
    }
    return newTask;
  }
}
