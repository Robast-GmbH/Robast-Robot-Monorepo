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
    [8.1125, -12.7593],
    [8.0814, -6.4185],
    [26.4201, -11.0498],
    [27.8188, -6.7293],
  ].map((pose) => Pose(x: pose[0], y: pose[1])).toList();

  final _navPoses = [
    [9.48, 4.01],
    [9.47, -2.0],
    [-8.74, 2.37],
    [-10.2, -1.69],
  ].map((pose) => Pose(x: pose[0], y: pose[1])).toList();

  late final Transformation _transformation = Transformation(
    sourcePoses: _navPoses,
    targetPoses: _rmfPoses,
  );

  Pose calculateNavGoal({required Pose pose}) {
    final scaledPose = Pose(x: pose.x * _scale, y: pose.y * _scale, yaw: pose.yaw + _transformation.rotation);
    return _transformation.tranformTargetToSource(scaledPose);
  }

  Pose calculateMapPosition({required Pose pose}) {
    final transformedPose = _transformation.tranformSourceToTarget(pose);
    final scaledPose = Pose(x: transformedPose.x / _scale, y: transformedPose.y / _scale, yaw: pose.yaw - _transformation.rotation);
    return scaledPose;
  }
}
