import 'package:web_frontend/models/dropoff_place_submodule_assignment.dart';

class TaskCreationController {
  // Fields for Patrol Task
  int? rounds;
  final List<String> places = [];

  // Fields for Delivery Task
  String? pickupPlaceID;
  String? dropoffPlaceID;
  String? submoduleID;

  // Fields for Multi Dropoff Task
  final List<DropoffPlaceSubmoduleAssignment> dropoffPlaceSubmoduleAssignments = [
    DropoffPlaceSubmoduleAssignment(),
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
    return pickupPlaceID != null && dropoffPlaceID != null && submoduleID != null;
  }

  bool _validateMultiDropOffTask() {
    return dropoffPlaceSubmoduleAssignments.isNotEmpty && dropoffPlaceSubmoduleAssignments.every((assignment) => assignment.isValid());
  }
}
