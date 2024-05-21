import 'dart:math';

class Point {
  final double x;
  final double y;

  Point(this.x, this.y);
}

Point centroid(List<Point> points) {
  double sumX = 0, sumY = 0;
  for (var point in points) {
    sumX += point.x;
    sumY += point.y;
  }
  return Point(sumX / points.length, sumY / points.length);
}

Point translate(Point point, Point translation) {
  return Point(point.x - translation.x, point.y - translation.y);
}

double calculateRotation(List<Point> points1, List<Point> points2) {
  double sum = 0;
  for (int i = 0; i < points1.length; i++) {
    sum += points1[i].x * points2[i].y - points1[i].y * points2[i].x;
  }
  return atan2(sum, dotProduct(points1, points2));
}

double dotProduct(List<Point> points1, List<Point> points2) {
  double sum = 0;
  for (int i = 0; i < points1.length; i++) {
    sum += points1[i].x * points2[i].x + points1[i].y * points2[i].y;
  }
  return sum;
}

Point rotate(Point point, double angle) {
  return Point(
    cos(angle) * point.x - sin(angle) * point.y,
    sin(angle) * point.x + cos(angle) * point.y,
  );
}

// # rmf:
// #   [
// #     [23.1098,-10.9255],
// #     [20.6076,-8.2368],
// #     [26.4201,-11.0498],
// #     [8.1125,-12.7593],
// #   ]

// # robot: [[0.428,-1.62], [-2,-4.31], [0.423, 1.73], [2.57, -16.8]]

void maind() {
  var points1 = [Point(23.1098, -10.9255), Point(20.6076, -8.2368), Point(26.4201, -11.0498), Point(8.1125, -12.7593)];
  var points2 = [Point(0.428, -1.62), Point(-2, -4.31), Point(0.423, 1.73), Point(2.57, -16.8)];

  var centroid1 = centroid(points1);
  var centroid2 = centroid(points2);

  var translatedPoints1 = points1.map((p) => translate(p, centroid1)).toList();
  var translatedPoints2 = points2.map((p) => translate(p, centroid2)).toList();

  var rotation = calculateRotation(translatedPoints1, translatedPoints2);

  final testPoint = Point(0.428, -1.62);
  final translatedPoint = translate(testPoint, centroid2);
  final rotatedTestPoint = rotate(translatedPoint, -rotation);
  final finalTestPoint = translate(rotatedTestPoint, Point(-centroid1.x, -centroid1.y));
  print('${finalTestPoint.x}, ${finalTestPoint.y}');

  var rotatedPoints1 = translatedPoints1.map((p) => rotate(p, rotation)).toList();

  var finalPoints1 = rotatedPoints1.map((p) => translate(p, Point(-centroid2.x, -centroid2.y))).toList();

  print(finalPoints1.map((e) => '(${e.x}, ${e.y})').join(', '));
}

class Transformation {
  Transformation({required List<Point> sourcePoints, required List<Point> targetPoints}) {
    sourceCentroid = centroid(sourcePoints);
    targetCentroid = centroid(targetPoints);
    rotation = calculateRotation(
      sourcePoints.map((p) => translate(p, sourceCentroid)).toList(),
      targetPoints.map((p) => translate(p, targetCentroid)).toList(),
    );
  }

  late final Point sourceCentroid;
  late final Point targetCentroid;
  late final double rotation;

  Point tranformSourceToTarget(Point point) {
    return translate(rotate(translate(point, sourceCentroid), rotation), Point(-targetCentroid.x, -targetCentroid.y));
  }

  Point tranformTargetToSource(Point point) {
    return translate(rotate(translate(point, targetCentroid), -rotation), Point(-sourceCentroid.x, -sourceCentroid.y));
  }

  Point centroid(List<Point> points) {
    double sumX = 0, sumY = 0;
    for (var point in points) {
      sumX += point.x;
      sumY += point.y;
    }
    return Point(sumX / points.length, sumY / points.length);
  }

  Point translate(Point point, Point translation) {
    return Point(point.x - translation.x, point.y - translation.y);
  }

  double calculateRotation(List<Point> points1, List<Point> points2) {
    double sum = 0;
    for (int i = 0; i < points1.length; i++) {
      sum += points1[i].x * points2[i].y - points1[i].y * points2[i].x;
    }
    return atan2(sum, dotProduct(points1, points2));
  }

  double dotProduct(List<Point> points1, List<Point> points2) {
    double sum = 0;
    for (int i = 0; i < points1.length; i++) {
      sum += points1[i].x * points2[i].x + points1[i].y * points2[i].y;
    }
    return sum;
  }

  Point rotate(Point point, double angle) {
    return Point(
      cos(angle) * point.x - sin(angle) * point.y,
      sin(angle) * point.x + cos(angle) * point.y,
    );
  }
}

void main() {
  var points1 = [Point(23.1098, -10.9255), Point(20.6076, -8.2368), Point(26.4201, -11.0498), Point(8.1125, -12.7593)];
  var points2 = [Point(0.428, -1.62), Point(-2, -4.31), Point(0.423, 1.73), Point(2.57, -16.8)];
  final transformation = Transformation(sourcePoints: points1, targetPoints: points2);
  final test1 = transformation.tranformSourceToTarget(Point(23.1098, -10.9255));
  final test2 = transformation.tranformTargetToSource(Point(0.428, -1.62));
  print('${test1.x}, ${test1.y}');
  print('${test2.x}, ${test2.y}');
}
