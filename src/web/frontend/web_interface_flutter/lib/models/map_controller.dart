import 'dart:ui';

class MapController {
  Offset? position;

  Offset getScaledPoint(double scale) {
    return Offset(position?.dx ?? 0 * scale, position?.dy ?? 0 * scale);
  }
}
