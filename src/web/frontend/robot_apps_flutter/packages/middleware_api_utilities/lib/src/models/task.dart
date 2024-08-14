import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:middleware_api_utilities/src/models/subtask.dart';
import 'package:uuid/uuid.dart';

class Task {
  Task({
    required this.id,
    required this.name,
    required this.status,
    required this.assigneeName,
    required this.requirements,
    required this.subtasks,
    required this.isMonolithic,
    required this.earliestStartTime,
    required this.priority,
  });

  factory Task.fromJson(Map<String, dynamic> json) {
    return Task(
      id: json['id'] as String,
      name: json['name'] as String,
      status: json['status'] as String,
      assigneeName: json['assignee_name'] as String,
      requirements: json['requirements'] as Map<String, dynamic>,
      subtasks: (json['subtasks'] as List<Map<String, dynamic>>).map(SubTask.fromJson).toList(),
      isMonolithic: json['is_monolithic'] as bool,
      earliestStartTime: json['earliest_start_time'] as int,
      priority: json['priority'] as int,
    );
  }

  factory Task.delivery({
    required int requiredDrawerType,
    required String pickupTargetID,
    required Map<String, int> pickupItemsByChange,
    required List<String> senderUserIDs,
    required List<String> senderUserGroups,
    required String dropoffTargetID,
    required Map<String, int> dropoffItemsByChange,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
  }) {
    final taskID = const Uuid().v4();
    final pickupTaskID = const Uuid().v4();
    final dropoffTaskID = const Uuid().v4();
    return Task(
      id: taskID,
      name: 'delivery',
      status: 'unassigned',
      assigneeName: '',
      requirements: {
        'required_drawer_type': requiredDrawerType,
      },
      subtasks: [
        SubTask.drawerProcess(
          id: pickupTaskID,
          parentID: taskID,
          requiresTaskID: null,
          targetID: pickupTargetID,
          requiredUserIDs: senderUserIDs,
          requiredUserGroups: senderUserGroups,
          itemsByChange: pickupItemsByChange,
        ),
        SubTask.drawerProcess(
          id: dropoffTaskID,
          parentID: taskID,
          requiresTaskID: pickupTaskID,
          targetID: dropoffTargetID,
          requiredUserIDs: recipientUserIDs,
          requiredUserGroups: recipientUserGroups,
          itemsByChange: dropoffItemsByChange,
        ),
      ],
      isMonolithic: false,
      earliestStartTime: DateTime.now().millisecondsSinceEpoch ~/ 1000,
      priority: 0,
    );
  }

  factory Task.dropoff({
    required String robotName,
    required String targetID,
    required Map<String, int> itemsByChange,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
    required DrawerAddress drawerAddress,
  }) {
    final taskID = const Uuid().v4();
    return Task(
      id: taskID,
      name: 'dropoff',
      status: 'pending',
      assigneeName: robotName,
      requirements: {},
      subtasks: [
        SubTask.assignedDrawerProcess(
          id: const Uuid().v4(),
          parentID: taskID,
          requiresTaskID: null,
          targetID: targetID,
          requiredUserIDs: recipientUserIDs,
          requiredUserGroups: recipientUserGroups,
          itemsByChange: itemsByChange,
          drawerAddress: drawerAddress,
          assigneeName: robotName,
        ),
      ],
      isMonolithic: false,
      earliestStartTime: DateTime.now().millisecondsSinceEpoch ~/ 1000,
      priority: 0,
    );
  }

  final String id;
  final String name;
  final String status;
  final String assigneeName;
  final Map<String, dynamic> requirements;
  final List<SubTask> subtasks;
  final bool isMonolithic;
  final int earliestStartTime;
  final int priority;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'status': status,
      'assignee_name': assigneeName,
      'requirements': requirements,
      'subtasks': subtasks.map((subtask) => subtask.toJson()).toList(),
      'is_monolithic': isMonolithic,
      'earliest_start_time': earliestStartTime,
      'priority': priority,
    };
  }
}
