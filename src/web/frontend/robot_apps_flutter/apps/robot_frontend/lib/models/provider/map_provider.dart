import 'package:flutter/material.dart';

class MapProvider extends ChangeNotifier {
  /// Keys are the stations and values the rooms.
  final Map<String, List<String>> _locations = {
    'HNO': [
      'Raum 1',
      'Raum 2',
      'Apotheke',
      'Rezeption',
      'Wartezimmer',
      'OP',
    ],
    'Psychiatrie': [
      'Raum 1',
      'Raum 2',
      'Rezeption',
      'Wartezimmer',
      'Gummizelle',
      'OP',
    ],
  };

  Map<String, List<String>> get locations => _locations;
}
