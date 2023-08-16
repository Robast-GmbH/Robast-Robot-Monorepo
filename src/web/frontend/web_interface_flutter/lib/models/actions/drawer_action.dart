import 'package:web_interface_flutter/models/actions/robot_action.dart';

class DrawerAction extends RobotAction {
  final int drawerID;
  final int moduleID;
  final List<String> lockedFor;
  final type = "OPEN_DRAWER";

  DrawerAction({
    required this.drawerID,
    required this.moduleID,
    required this.lockedFor,
  });

  static DrawerAction fromJson(Map<String, dynamic> data) {
    return DrawerAction(
      drawerID: data["drawer_id"],
      moduleID: data["module_id"],
      lockedFor: data["locked_for"],
    );
  }
}
