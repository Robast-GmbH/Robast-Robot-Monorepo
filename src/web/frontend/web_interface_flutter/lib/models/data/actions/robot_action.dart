import 'package:web_interface_flutter/models/data/actions/drawer_action.dart';
import 'package:web_interface_flutter/models/data/actions/new_user.dart';
import 'package:web_interface_flutter/models/data/actions/waypoint.dart';

class RobotAction {
  final bool isFinished;
  final int step;

  RobotAction({
    required this.isFinished,
    required this.step,
  });

  static RobotAction fromJson(Map<String, dynamic> data) {
    switch (data["type"]) {
      case DrawerAction.type:
        return DrawerAction.fromJson(
          data["action"],
          data["finished"],
          data["step"],
        );
      case Waypoint.type:
        return Waypoint.fromJson(
          data["action"],
          data["finished"],
          data["step"],
        );
      case NewUser.type:
        return NewUser.fromJson(
          data["action"],
          data["finished"],
          data["step"],
        );
    }
    throw TypeError();
  }
}
