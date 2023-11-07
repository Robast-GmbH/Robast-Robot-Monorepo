import 'package:web_interface_flutter/models/data/actions/robot_action.dart';

class DrawerAction extends RobotAction {
  final int drawerID;
  final int moduleID;
  final List<String> lockedFor;
  static const type = "DRAWER";

  DrawerAction({
    required this.drawerID,
    required this.moduleID,
    required this.lockedFor,
    required super.isFinished,
    required super.step,
  });

  static DrawerAction fromJson(Map<String, dynamic> data, bool isFinished, int step) {
    return DrawerAction(
      drawerID: data["drawer_id"] as int,
      moduleID: data["module_id"] as int,
      lockedFor: data["locked_for"].cast<String>() as List<String>,
      isFinished: isFinished,
      step: step,
    );
  }
}
