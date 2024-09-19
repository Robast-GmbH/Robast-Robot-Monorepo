class DeliveryTimeController {
  DateTime? value;

  int timeAsSecondsSinceEpoch() {
    final timeStamp = value ?? DateTime.now();
    return timeStamp.millisecondsSinceEpoch ~/ 1000;
  }
}
