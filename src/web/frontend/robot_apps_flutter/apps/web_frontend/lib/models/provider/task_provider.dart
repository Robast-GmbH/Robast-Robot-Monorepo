import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';

class RMFProvider extends ChangeNotifier {
  List<Task> tasks = [];
  BuildingMap? _buildingMap;
  final Map<String, String> _interruptTokens = {};
  Timer? _tasksUpdateTimer;

  final _middlewareApi = MiddlewareApiUtilities();

  Future<void> updateBuildingMap() async {
    try {
      // _buildingMap = await _rmfApi.getBuildingMap();
      notifyListeners();
    } catch (e) {
      debugPrint('Error getting building map: $e');
    }
  }

  Future<void> updateTasks() async {
    try {
      //  tasks = await _rmfApi.getTasks();
      notifyListeners();
    } catch (e) {
      debugPrint('Error getting tasks: $e');
    }
  }

  void startPeriodicTasksUpdate() {
    updateTasks();
    _tasksUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      debugPrint('Update Tasks');
      await updateTasks();
    });
  }

  void stopPeriodicTasksUpdate() {
    _tasksUpdateTimer?.cancel();
  }

  List<Vertice> getVertices() {
    return _buildingMap?.levels.first.vertices ?? [];
  }

  List<String> getPickupLocations() {
    return getVertices().where((element) => element.isDispenser).map((e) => e.name).toList();
  }

  List<String> getDropoffLocations() {
    return getVertices().where((element) => element.isIngestor).map((e) => e.name).toList();
  }

  Future<void> dispatchTask({required String taskType, required TaskCreationController controller}) async {
    if (taskType == 'Patrol') {
      await dispatchPatrolTask(
        places: controller.places,
        rounds: controller.rounds!,
      );
    } else if (taskType == 'Delivery') {
      await dispatchDeliveryTask(
        pickup: controller.pickupPlaceID!,
        dropoff: controller.dropoffPlaceID!,
        submoduleID: controller.submoduleID!,
      );
    } else if (taskType == 'Multi Dropoff') {
      await dispatchPatrolTask(places: ['floor'], rounds: 1);
      for (final dropoffPlaceSubmoduleAssignment in controller.dropoffPlaceSubmoduleAssignments) {
        await dispatchDropOffTask(
          dropoff: dropoffPlaceSubmoduleAssignment.dropoffPlaceID!,
          submoduleID: dropoffPlaceSubmoduleAssignment.submoduleID!,
        );
      }
    }
  }

  Future<void> dispatchDeliveryTask({
    required String pickup,
    required String dropoff,
    required String submoduleID,
  }) async {
    try {} catch (e) {
      debugPrint('Error dispatching delivery task: $e');
    }
  }

  Future<void> dispatchDropOffTask({
    required String dropoff,
    required String submoduleID,
  }) async {
    try {} catch (e) {
      debugPrint('Error dispatching dropoff task: $e');
    }
  }

  Future<void> dispatchPatrolTask({
    required List<String> places,
    required int rounds,
  }) async {
    try {} catch (e) {
      debugPrint('Error dispatching patrol task: $e');
    }
  }

  Future<void> interruptTask({
    required String taskID,
  }) async {
    try {
      // final token = await _rmfApi.interruptTask(taskID: taskID);
      // debugPrint('Interrupt token: $token');
      // _interruptTokens[taskID] = token;
    } catch (e) {
      debugPrint('Error interrupting task: $e');
    }
  }

  Future<bool> resumeTask({
    required String taskID,
  }) async {
    try {
      // return await _rmfApi.resumeTask(taskID: taskID, token: _interruptTokens[taskID]!);
      return true;
    } catch (e) {
      debugPrint('Error resuming task: $e');
      return false;
    }
  }
}
