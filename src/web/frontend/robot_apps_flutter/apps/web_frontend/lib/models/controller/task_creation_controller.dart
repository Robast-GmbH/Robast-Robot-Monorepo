class TaskCreationController {
  // Fields for Patrol Task
  int? rounds;
  List<String> places = [];

  // Fields for Delivery Task
  String? pickupNode;
  String? dropoffNode;
  String? drawerID;

  bool validateTask({required String type}) {
    if (type == 'Patrol') {
      return _validatePatrolTask();
    } else if (type == 'Delivery') {
      return _validateDeliveryTask();
    }
    return false;
  }

  bool _validatePatrolTask() {
    return rounds != null && rounds! > 0 && places.isNotEmpty && places.every((element) => element.isNotEmpty);
  }

  bool _validateDeliveryTask() {
    return pickupNode != null && dropoffNode != null && drawerID != null;
  }
}
