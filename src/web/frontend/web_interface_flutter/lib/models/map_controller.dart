import 'dart:ui';

class MapController {
  final Offset mapOrigin = const Offset(-23.3, -7.33);
  final double resolution = 0.05;
  final int mapHeight = 326;
  final int mapWidth = 809;
  final points = <Offset>[];
  final yaws = <double>[];

  Offset getRobotOriginAsOffset() {
    return Offset(-mapOrigin.dx / resolution, mapHeight + mapOrigin.dy / resolution);
  }

  List<Offset> positionsAsNavPoints() {
    final scaledPositions = getScaledPoints(resolution);
    return scaledPositions.map((e) => Offset(mapOrigin.dx + e.dx, mapOrigin.dy + mapHeight * resolution - e.dy)).toList();
  }

  List<Offset> getScaledPoints(double scale) {
    return points.map((e) => Offset((e.dx) * scale, (e.dy) * scale)).toList();
  }
}
