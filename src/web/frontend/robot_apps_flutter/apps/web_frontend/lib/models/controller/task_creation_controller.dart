class TaskCreationController {
  // Fields for Patrol Task
  int? rounds;
  final List<String> places = [];

  // Fields for Delivery Task
  String? pickupNode;
  String? dropoffNode;
  String? drawerID;

  // Fields for Multi Dropoff Task
  final List<List<String?>> dropoffNodeDrawerAssignments = [
    [null, null]
  ];

  bool validateTask({required String type}) {
    if (type == 'Patrol') {
      return _validatePatrolTask();
    } else if (type == 'Delivery') {
      return _validateDeliveryTask();
    } else if (type == 'Multi Dropoff') {
      return _validateMultiDropOffTask();
    }
    return false;
  }

  bool _validatePatrolTask() {
    return rounds != null && rounds! > 0 && places.isNotEmpty && places.every((element) => element.isNotEmpty);
  }

  bool _validateDeliveryTask() {
    return pickupNode != null && dropoffNode != null && drawerID != null;
  }

  bool _validateMultiDropOffTask() {
    return dropoffNodeDrawerAssignments.isNotEmpty && dropoffNodeDrawerAssignments.every((element) => element[0] != null && element[1] != null);
  }
}
