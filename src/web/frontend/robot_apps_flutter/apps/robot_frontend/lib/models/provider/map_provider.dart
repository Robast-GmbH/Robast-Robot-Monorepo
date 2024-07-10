import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class MapProvider extends ChangeNotifier {
  /// Keys are the stations and values the rooms.
  Map<String, List<String>> _locations = {};

  Map<String, List<String>> get locations => _locations;
  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<void> fetchBuildingMap() async {
    final buildingMap = await _middlewareApiUtilities.getMap();
    if (buildingMap == null) {
      return;
    }
    final mapLocations = <String, List<String>>{};
    for (final level in buildingMap.levels) {
      final stations = <String>[];
      for (final station in level.vertices) {
        stations.add(station.name);
      }
      mapLocations[level.name] = stations;
    }
    _locations = mapLocations;
    notifyListeners();
  }
}
