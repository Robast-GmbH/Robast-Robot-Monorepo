import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class TaskProvider extends ChangeNotifier {
  TaskProvider({required String prefix}) {
    _middlewareApi = MiddlewareApiUtilities();
    _middlewareApi.setPrefix(prefix: prefix);
  }

  void initMiddlewarAPI({required String prefix}) {
    _middlewareApi.setPrefix(prefix: prefix);
  }

  List<Task> tasks = [];
  BuildingMap? _buildingMap;
  Timer? _tasksUpdateTimer;

  late final MiddlewareApiUtilities _middlewareApi;

  Future<bool> createDeliveryTaskRequest({
    required int requiredSubmoduleType,
    required String pickupTargetID,
    required int pickupEarliestStartTime,
    required List<String> senderUserIDs,
    required List<String> senderUserGroups,
    required String dropoffTargetID,
    required int dropoffEarliestStartTime,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
    required Map<String, int> itemsByChange,
  }) async {
    final dropoffItemsByChange = itemsByChange.map((key, value) => MapEntry(key, -value));
    final task = Task.delivery(
      requiredSubmoduleType: requiredSubmoduleType,
      pickupTargetID: pickupTargetID,
      pickupEarliestStartTime: pickupEarliestStartTime,
      pickupItemsByChange: itemsByChange,
      senderUserIDs: senderUserIDs,
      senderUserGroups: senderUserGroups,
      dropoffTargetID: dropoffTargetID,
      dropoffEarliestStartTime: dropoffEarliestStartTime,
      dropoffItemsByChange: dropoffItemsByChange,
      recipientUserIDs: recipientUserIDs,
      recipientUserGroups: recipientUserGroups,
    );
    final wasSuccessful = await _middlewareApi.tasks.postTaskRequest(task: task);
    return wasSuccessful;
  }

  Future<void> updateTasks() async {
    try {
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

  Future<RobotTaskStatus?> getRobotTasks({required String robotName}) async {
    return _middlewareApi.tasks.getRobotTasks(robotName: robotName);
  }

  Future<List<Task>?> fetchTasks({required String robotName, required int limit, required int offset}) async {
    final tasks = await _middlewareApi.tasks.getFinishedTasks(robotName: robotName, limit: limit, offset: offset);
    return tasks;
  }
}
