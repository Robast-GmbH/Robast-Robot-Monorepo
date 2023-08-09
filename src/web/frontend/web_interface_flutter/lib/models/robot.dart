class Robot {
  final String name;
  final String fleetName;
  final double x;
  final double y;
  final double yaw;
  final int taskID;
  Robot({
    required this.name,
    required this.fleetName,
    required this.x,
    required this.y,
    required this.yaw,
    required this.taskID,
  });

  static Robot fromJson(Map<String, dynamic> data) {
    return Robot(
      name: data["robot_name"],
      fleetName: data["fleet_name"],
      x: data["x_pose"],
      y: data["y_pose"],
      yaw: data["yaw_pose"],
      taskID: data["task_id"],
    );
  }

  static Robot mock(String name) {
    return Robot(
      name: name,
      fleetName: "Flotte 1",
      x: 1.46,
      y: -2.87,
      yaw: 0,
      taskID: 0,
    );
  }
}
