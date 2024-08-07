import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class TaskProvider extends ChangeNotifier {
  RobotTaskStatus? _robotTaskStatus;
  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Task? get activeTask => _robotTaskStatus?.activeTask;
  List<Task> get queuedTasks => _robotTaskStatus?.queuedTasks ?? [];

  Future<void> fetchRobotTaskStatus({required String robotName}) async {
    _robotTaskStatus = await _middlewareApiUtilities.tasks.getRobotTasks(robotName: robotName);
    notifyListeners();
  }

  Future<bool> createTaskRequest({
    required int requiredDrawerType,
    required Map<String, int> itemsByChange,
    required List<String> senderAuthUsers,
    required List<String> senderAuthUserGroups,
    required List<String> recipientAuthUsers,
    required List<String> recipientAuthUserGroups,
    String? startID,
    String? targetID,
  }) async {
    final wasSuccessful = await _middlewareApiUtilities.tasks.postTaskRequest(
      requiredDrawerType: requiredDrawerType,
      itemsByChange: itemsByChange,
      startID: startID,
      targetID: targetID,
      senderAuthUsers: senderAuthUsers,
      senderAuthUserGroups: senderAuthUserGroups,
      recipientAuthUsers: recipientAuthUsers,
      recipientAuthUserGroups: recipientAuthUserGroups,
    );
    return wasSuccessful;
  }
}
