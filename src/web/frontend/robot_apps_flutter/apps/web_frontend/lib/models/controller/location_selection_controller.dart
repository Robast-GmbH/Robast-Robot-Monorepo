class LocationSelectionController {
  String? _station;
  String? _room;

  void setStation(String? value) {
    _station = value;
    _room = null;
  }

  String? get station => _station;

  void setRoom(String? value) {
    if (station != null) _room = value;
  }

  String? get room => _room;
}
