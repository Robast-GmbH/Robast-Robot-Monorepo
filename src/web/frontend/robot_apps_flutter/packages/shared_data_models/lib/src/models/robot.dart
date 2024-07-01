import 'package:shared_data_models/src/models/pose.dart';

class Robot {
  Robot({
    required this.name,
    required this.fleetName,
    required this.pose,
    required this.batteryLevel,
  });

  Robot.mock({required this.name})
      : fleetName = 'Flotte 1',
        batteryLevel = 0,
        pose = Pose(
          x: 1.46,
          y: -2.87,
        );

  Robot.fromJson({required Map<String, dynamic> data})
      : name = data['robot_name'] as String,
        fleetName = data['fleet_name'] as String,
        batteryLevel = data['battery_level'] as double,
        pose = Pose(
          x: data['x_pose'] as double,
          y: data['y_pose'] as double,
          yaw: data['yaw_pose'] as double,
        );

  final String name;
  final String fleetName;
  final double batteryLevel;
  final Pose pose;
}
