class Pose {
  Pose({
    required this.x,
    required this.y,
    this.yaw = 0.0,
  });

  final double x;
  final double y;
  final double yaw;

  @override
  String toString() {
    return 'Pose(x: $x, y: $y, yaw: $yaw)';
  }
}
