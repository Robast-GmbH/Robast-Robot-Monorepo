import 'package:web_interface_flutter/models/actions/robot_action.dart';

class Task {
  final int id;
  final List<RobotAction> actions;

  Task({
    required this.id,
    required this.actions,
  });

  static Task fromJson(Map<String, dynamic> data) {
    (data["actions"] as List<Map<String, dynamic>>).sort(
      (a, b) => a["step"] < b["step"],
    );
    return Task(
      id: data["id"],
      actions: (data["actions"] as List<Map<String, dynamic>>).map((e) => RobotAction.fromJson(e["action"])).toList(),
    );
  }
}
