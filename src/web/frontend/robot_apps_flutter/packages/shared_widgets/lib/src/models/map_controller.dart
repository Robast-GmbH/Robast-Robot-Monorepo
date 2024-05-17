import 'dart:ui';

class MapController {
  MapController({required this.mapPath});

  final Offset mapOrigin = const Offset(2000, -660.0);
  final int mapHeight = 1340;
  final int mapWidth = 4364;
  final String mapPath;
  final points = <Offset>[];
  final yaws = <double>[];

  Offset getRobotOriginAsOffset() {
    //return Offset(855, 545);
    return Offset(mapOrigin.dx, mapHeight + mapOrigin.dy);
  }
}
