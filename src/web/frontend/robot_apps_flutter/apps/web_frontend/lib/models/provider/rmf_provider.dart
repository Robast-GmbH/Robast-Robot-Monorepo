import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:rmf_api/rmf_api.dart';

class RMFProvider extends ChangeNotifier {
  List<Task> tasks = [];
  BuildingMap? _buildingMap;
  final Map<String, String> _interruptTokens = {};
  Timer? _tasksUpdateTimer;

  final _rmfApi = const RmfApi(prefix: 'http://10.10.23.6:8000');

  Future<void> updateBuildingMap() async {
    try {
      _buildingMap = await _rmfApi.getBuildingMap();
      notifyListeners();
    } catch (e) {
      debugPrint('Error getting building map: $e');
    }
  }

  Future<void> updateTasks() async {
    try {
      tasks = await _rmfApi.getTasks();
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

  Future<void> dispatchDeliveryTask({
    required String pickup,
    required String dropoff,
    required String drawerID,
  }) async {
    try {
      await _rmfApi.dispatchDeliveryTask(
        pickup: pickup,
        dropoff: dropoff,
        drawerID: drawerID,
      );
    } catch (e) {
      debugPrint('Error dispatching delivery task: $e');
    }
  }

  Future<void> dispatchPatrolTask({
    required List<String> places,
    required int rounds,
  }) async {
    try {
      await _rmfApi.dispatchPatrolTask(
        places: places,
        rounds: rounds,
      );
    } catch (e) {
      debugPrint('Error dispatching patrol task: $e');
    }
  }

  Future<void> interruptTask({
    required String taskID,
  }) async {
    try {
      final token = await _rmfApi.interruptTask(taskID: taskID);
      debugPrint('Interrupt token: $token');
      _interruptTokens[taskID] = token;
    } catch (e) {
      debugPrint('Error interrupting task: $e');
    }
  }

  Future<bool> resumeTask({
    required String taskID,
  }) async {
    try {
      return await _rmfApi.resumeTask(taskID: taskID, token: _interruptTokens[taskID]!);
    } catch (e) {
      debugPrint('Error resuming task: $e');
      return false;
    }
  }
}
