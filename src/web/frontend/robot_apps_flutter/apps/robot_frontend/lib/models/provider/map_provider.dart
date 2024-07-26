import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class MapProvider extends ChangeNotifier {
  Map<String, List<String>> _roomsByStations = {};

  Map<String, List<String>> get roomsByStations => _roomsByStations;
  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<void> fetchBuildingMap() async {
    final buildingMap = await _middlewareApiUtilities.getMap();
    if (buildingMap == null) {
      return;
    }
    final tempRoomsByStations = <String, List<String>>{};
    for (final level in buildingMap.levels) {
      final stations = <String>[];
      for (final station in level.vertices) {
        stations.add(station.name);
      }
      tempRoomsByStations[level.name] = stations;
    }
    _roomsByStations = tempRoomsByStations;
    notifyListeners();
  }
}
