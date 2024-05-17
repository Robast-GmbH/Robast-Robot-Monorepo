import 'dart:math';

class Point {
  Point(this.x, this.y);
  final double x;
  final double y;
}

class Transformation {
  Transformation({required List<Point> sourcePoints, required List<Point> targetPoints}) {
    _sourceCentroid = _centroid(sourcePoints);
    _targetCentroid = _centroid(targetPoints);
    _rotation = _calculateRotation(
      sourcePoints.map((p) => _translate(p, _sourceCentroid)).toList(),
      targetPoints.map((p) => _translate(p, _targetCentroid)).toList(),
    );
  }

  late final Point _sourceCentroid;
  late final Point _targetCentroid;
  late final double _rotation;

  Point tranformSourceToTarget(Point point) {
    return _translate(_rotate(_translate(point, _sourceCentroid), _rotation), Point(-_targetCentroid.x, -_targetCentroid.y));
  }

  Point tranformTargetToSource(Point point) {
    return _translate(_rotate(_translate(point, _targetCentroid), -_rotation), Point(-_sourceCentroid.x, -_sourceCentroid.y));
  }

  Point _centroid(List<Point> points) {
    var sumX = 0.0;
    var sumY = 0.0;
    for (final point in points) {
      sumX += point.x;
      sumY += point.y;
    }
    return Point(sumX / points.length, sumY / points.length);
  }

  Point _translate(Point point, Point translation) {
    return Point(point.x - translation.x, point.y - translation.y);
  }

  double _calculateRotation(List<Point> points1, List<Point> points2) {
    var sum = 0.0;
    for (var i = 0; i < points1.length; i++) {
      sum += points1[i].x * points2[i].y - points1[i].y * points2[i].x;
    }
    return atan2(sum, _dotProduct(points1, points2));
  }

  double _dotProduct(List<Point> points1, List<Point> points2) {
    var sum = 0.0;
    for (var i = 0; i < points1.length; i++) {
      sum += points1[i].x * points2[i].x + points1[i].y * points2[i].y;
    }
    return sum;
  }

  Point _rotate(Point point, double angle) {
    return Point(
      cos(angle) * point.x - sin(angle) * point.y,
      sin(angle) * point.x + cos(angle) * point.y,
    );
  }
}
