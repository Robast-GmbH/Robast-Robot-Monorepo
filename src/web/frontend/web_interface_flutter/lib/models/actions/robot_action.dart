import 'package:web_interface_flutter/models/actions/drawer_action.dart';
import 'package:web_interface_flutter/models/actions/new_user.dart';
import 'package:web_interface_flutter/models/actions/waypoint.dart';

class RobotAction {
  static RobotAction fromJson(Map<String, dynamic> data) {
    switch (data["type"]) {
      case DrawerAction.type:
        return DrawerAction.fromJson(data["action"]);
      case Waypoint.type:
        return Waypoint.fromJson(data["action"]);
      case NewUser.type:
        return NewUser.fromJson(data["action"]);
    }
    throw TypeError();
  }
}
