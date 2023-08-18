import 'package:web_interface_flutter/models/actions/robot_action.dart';

class NewUser extends RobotAction {
  final int userID;

  static const type = "NEW_USER";

  NewUser({
    required this.userID,
  });

  static NewUser fromJson(Map<String, dynamic> data) {
    return NewUser(userID: data["user_id"]);
  }
}
