import 'dart:math';

import 'package:shared_data_models/shared_data_models.dart';

class Transformation {
  Transformation({required List<Pose> sourcePoses, required List<Pose> targetPoses}) {
    sourceCentroid = centroid(sourcePoses);
    targetCentroid = centroid(targetPoses);
    rotation = calculateRotation(
      sourcePoses.map((p) => translate(p, sourceCentroid)).toList(),
      targetPoses.map((p) => translate(p, targetCentroid)).toList(),
    );
  }

  late final Pose sourceCentroid;
  late final Pose targetCentroid;
  late final double rotation;

  Pose tranformSourceToTarget(Pose pose) {
    return translate(
      rotate(translate(pose, sourceCentroid), rotation),
      Pose(x: -targetCentroid.x, y: -targetCentroid.y, yaw: pose.yaw),
    );
  }

  Pose tranformTargetToSource(Pose pose) {
    return translate(
      rotate(translate(pose, targetCentroid), -rotation),
      Pose(x: -sourceCentroid.x, y: -sourceCentroid.y),
    );
  }

  Pose centroid(List<Pose> poses) {
    double sumX = 0, sumY = 0;
    for (var pose in poses) {
      sumX += pose.x;
      sumY += pose.y;
    }
    return Pose(x: sumX / poses.length, y: sumY / poses.length, yaw: 0.0);
  }

  Pose translate(Pose pose, Pose translation) {
    return Pose(x: pose.x - translation.x, y: pose.y - translation.y, yaw: 0.0);
  }

  double calculateRotation(List<Pose> poses1, List<Pose> poses2) {
    double sum = 0;
    for (int i = 0; i < poses1.length; i++) {
      sum += poses1[i].x * poses2[i].y - poses1[i].y * poses2[i].x;
    }
    return atan2(sum, dotProduct(poses1, poses2));
  }

  double dotProduct(List<Pose> poses1, List<Pose> poses2) {
    double sum = 0;
    for (int i = 0; i < poses1.length; i++) {
      sum += poses1[i].x * poses2[i].x + poses1[i].y * poses2[i].y;
    }
    return sum;
  }

  Pose rotate(Pose pose, double angle) {
    return Pose(x: cos(angle) * pose.x - sin(angle) * pose.y, y: sin(angle) * pose.x + cos(angle) * pose.y, yaw: 0.0);
  }
}
