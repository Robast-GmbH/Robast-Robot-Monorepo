import 'dart:math';

import 'package:shared_data_models/shared_data_models.dart';
import 'package:shared_data_models/src/models/transformation.dart';

class MapController {
  MapController({required this.mapPath});

  final int mapHeight = 1196;
  final int mapWidth = 2689;
  final double _scale = 0.0155;
  final String mapPath;
  final yaws = <double>[];
  final _rmfPoses = [
    Pose(x: 9.2936, y: -7.6929),
    Pose(x: 18.3853, y: -11.7025),
    Pose(x: 34.7035, y: -10.7701),
    Pose(x: 31.3, y: -12.6039),
  ];
  final _navPoses = [
    Pose(x: 8.28, y: -0.442),
    Pose(x: -0.951, y: 2.82),
    Pose(x: -17.2, y: 1.96),
    Pose(x: -13.6, y: 3.74),
  ];

  late final Transformation _transformation = Transformation(
    sourcePoses: _navPoses,
    targetPoses: _rmfPoses,
  );

  Pose calculateNavGoal({required Pose pose}) {
    final scaledPose = Pose(x: pose.x * _scale, y: pose.y * _scale);
    return _transformation.tranformTargetToSource(scaledPose);
  }

  Pose calculateMapPosition({required Pose pose}) {
    final transformedPose = _transformation.tranformSourceToTarget(pose);
    final scaledPose = Pose(x: transformedPose.x / _scale, y: transformedPose.y / _scale, yaw: pose.yaw - _transformation.rotation);
    return scaledPose;
  }

  double test(double rotation) {
    if (rotation > pi) {
      return rotation - 2 * pi;
    } else {
      return rotation;
    }
  }
}
