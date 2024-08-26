import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

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
    required int requiredSubmoduleType,
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
      requiredSubmoduleType: requiredSubmoduleType,
      pickupTargetID: pickupTargetID,
      pickupItemsByChange: itemsByChange,
      senderUserIDs: senderUserIDs,
      senderUserGroups: senderUserGroups,
      dropoffTargetID: dropoffTargetID,
      dropoffItemsByChange: dropoffItemsByChange,
      recipientUserIDs: recipientUserIDs,
      recipientUserGroups: recipientUserGroups,
    );
    final wasSuccessful = await _middlewareApiUtilities.tasks.postTaskRequest(task: task);
    return wasSuccessful;
  }

  Future<bool> createDirectDropoffTask({
    required String robotName,
    required String dropoffTargetID,
    required User? user,
    required List<String> userGroups,
    required Submodule submodule,
  }) async {
    final dropoffItemsByChange = submodule.itemsByCount.map((key, value) => MapEntry(key, -value));
    final task = Task.dropoff(
      robotName: 'rb_theron',
      targetID: dropoffTargetID,
      itemsByChange: dropoffItemsByChange,
      recipientUserIDs: user?.id != null ? [user!.id] : [],
      recipientUserGroups: userGroups,
      submoduleAddress: submodule.address,
    );

    final wasSuccessful = await _middlewareApiUtilities.tasks.postTaskRequest(task: task);
    return wasSuccessful;
  }
}
