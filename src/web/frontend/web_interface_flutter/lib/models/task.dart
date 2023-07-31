enum TaskTypes { delivery, move, drawer }

extension ToStringExtension on TaskTypes {
  static final nameMap = <TaskTypes, String>{
    TaskTypes.delivery: "Delivery",
    TaskTypes.drawer: "Drawer",
    TaskTypes.move: "Move",
  };
  String toShortString() {
    return nameMap[this] ?? "Error";
  }
}

class Task {
  final int id;
  final double xPose;
  final double yPose;
  final double yawPose;
  final bool finished;
  final int targetID;
  final int moduleID;
  final int drawerID;
  final String robotName;

  final int ownerID;

  Task({
    required this.id,
    required this.xPose,
    required this.yPose,
    required this.yawPose,
    required this.finished,
    required this.targetID,
    required this.moduleID,
    required this.drawerID,
    required this.robotName,
    required this.ownerID,
  });

  static Task fromJson(Map<String, dynamic> data) {
    return Task(
      id: data["id"],
      xPose: data["x_pose"],
      yPose: data["y_pose"],
      yawPose: data["yaw_pose"],
      finished: data["finished"],
      targetID: data["target_id"],
      moduleID: data["module_id"],
      drawerID: data["drawer_id"],
      robotName: data["robot_name"],
      ownerID: data["owner_id"],
    );
  }
}
