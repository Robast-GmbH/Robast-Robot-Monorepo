import 'package:middleware_api_utilities/src/models/subtask.dart';

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
    required String id,
    required int requiredDrawerType,
    required String pickupTaskID,
    required String pickupTargetID,
    required Map<String, int> pickupItemsByChange,
    required List<String> senderUserIDs,
    required List<String> senderUserGroups,
    required String dropoffTaskID,
    required String dropoffTargetID,
    required Map<String, int> dropoffItemsByChange,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
  }) {
    return Task(
      id: id,
      name: 'delivery',
      status: 'unassigned',
      assigneeName: '',
      requirements: {
        'required_drawer_type': requiredDrawerType,
      },
      subtasks: [
        SubTask.drawerProcess(
          id: pickupTaskID,
          parentID: id,
          requiresTaskID: null,
          targetID: pickupTargetID,
          requiredUserIDs: senderUserIDs,
          requiredUserGroups: senderUserGroups,
          itemsByChange: pickupItemsByChange,
        ),
        SubTask.drawerProcess(
          id: dropoffTaskID,
          parentID: id,
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
