import 'package:web_frontend/models/dropoff_place_drawer_assignment.dart';

class TaskCreationController {
  // Fields for Patrol Task
  int? rounds;
  final List<String> places = [];

  // Fields for Delivery Task
  String? pickupPlaceID;
  String? dropoffPlaceID;
  String? drawerID;

  // Fields for Multi Dropoff Task
  final List<DropoffPlaceDrawerAssignment> dropoffPlaceDrawerAssignments = [
    DropoffPlaceDrawerAssignment(),
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
    return pickupPlaceID != null && dropoffPlaceID != null && drawerID != null;
  }

  bool _validateMultiDropOffTask() {
    return dropoffPlaceDrawerAssignments.isNotEmpty && dropoffPlaceDrawerAssignments.every((assignment) => assignment.isValid());
  }
}
