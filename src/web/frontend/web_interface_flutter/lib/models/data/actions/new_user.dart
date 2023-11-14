import 'package:web_interface_flutter/models/data/actions/robot_action.dart';

class NewUser extends RobotAction {
  final int userID;

  static const type = "NEW_USER";

  NewUser({
    required this.userID,
    required super.isFinished,
    required super.step,
  });

  static NewUser fromJson(Map<String, dynamic> data, bool isFinished, int step) {
    return NewUser(
      userID: data["user_id"] as int,
      isFinished: isFinished,
      step: step,
    );
  }
}
