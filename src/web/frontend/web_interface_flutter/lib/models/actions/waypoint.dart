import 'package:web_interface_flutter/models/actions/robot_action.dart';

class Waypoint extends RobotAction {
  final double x;
  final double y;
  final double yaw;
  static const type = "NAVIGATION";

  Waypoint({
    required this.x,
    required this.y,
    required this.yaw,
  });

  static Waypoint fromJson(Map<String, dynamic> data) {
    return Waypoint(
      x: data["pose"]["x"],
      y: data["pose"]["y"],
      yaw: data["yaw"],
    );
  }
}
