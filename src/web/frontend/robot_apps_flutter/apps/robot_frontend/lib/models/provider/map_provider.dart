import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class MapProvider extends ChangeNotifier {
  Uint8List? mapImageCache;
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

  Future<Uint8List?> getMapImage() async {
    if (mapImageCache == null) {
      mapImageCache = await _middlewareApiUtilities.getMapImage();
    }
    return mapImageCache;
  }
}
