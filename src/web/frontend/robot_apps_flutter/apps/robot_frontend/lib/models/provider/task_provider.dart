import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:uuid/uuid.dart';

class TaskProvider extends ChangeNotifier {
  RobotTaskStatus? _robotTaskStatus;
  final _middlewareApiUtilities = MiddlewareApiUtilities();

  SubTask? get activeTask => _robotTaskStatus?.activeTask;
  List<SubTask> get queuedTasks => _robotTaskStatus?.queuedTasks ?? [];

  Future<void> fetchRobotTaskStatus({required String robotName}) async {
    _robotTaskStatus = await _middlewareApiUtilities.tasks.getRobotTasks(robotName: robotName);
    notifyListeners();
  }

  Future<bool> createDeliveryTaskRequest({
    required int requiredDrawerType,
    required String pickupTargetID,
    required List<String> senderUserIDs,
    required List<String> senderUserGroups,
    required String dropoffTargetID,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
    required Map<String, int> itemsByChange,
  }) async {
    final dropoffItemsByChange = itemsByChange.map((key, value) => MapEntry(key, -value));
    final task = Task.delivery(
      id: const Uuid().v4(),
      requiredDrawerType: requiredDrawerType,
      pickupTaskID: const Uuid().v4(),
      pickupTargetID: pickupTargetID,
      pickupItemsByChange: itemsByChange,
      senderUserIDs: senderUserIDs,
      senderUserGroups: senderUserGroups,
      dropoffTaskID: const Uuid().v4(),
      dropoffTargetID: dropoffTargetID,
      dropoffItemsByChange: dropoffItemsByChange,
      recipientUserIDs: recipientUserIDs,
      recipientUserGroups: recipientUserGroups,
    );
    final wasSuccessful = await _middlewareApiUtilities.tasks.postTaskRequest(task: task);
    return wasSuccessful;
  }
}
