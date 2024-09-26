import 'package:shared_data_models/src/models/submodule_address.dart';
import 'package:shared_data_models/src/models/subtask.dart';
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
      subtasks: (json['subtasks'] as List<dynamic>).map((task) => SubTask.fromJson(task as Map<String, dynamic>)).toList(),
      isMonolithic: json['is_monolithic'] as bool,
      earliestStartTime: json['earliest_start_time'] as int,
      priority: json['priority'] as int,
    );
  }

  factory Task.delivery({
    required int requiredSubmoduleType,
    required String pickupTargetID,
    required int pickupEarliestStartTime,
    required Map<String, int> pickupItemsByChange,
    required List<String> senderUserIDs,
    required List<String> senderUserGroups,
    required String dropoffTargetID,
    required int dropoffEarliestStartTime,
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
        'required_submodule_type': requiredSubmoduleType,
      },
      subtasks: [
        SubTask.submoduleProcess(
            id: pickupTaskID,
            name: 'Abholung',
            parentID: taskID,
            requiresTaskID: null,
            targetID: pickupTargetID,
            requiredUserIDs: senderUserIDs,
            requiredUserGroups: senderUserGroups,
            itemsByChange: pickupItemsByChange,
            earliestStartTime: pickupEarliestStartTime,),
        SubTask.submoduleProcess(
            id: dropoffTaskID,
            name: 'Zustellung',
            parentID: taskID,
            requiresTaskID: pickupTaskID,
            targetID: dropoffTargetID,
            requiredUserIDs: recipientUserIDs,
            requiredUserGroups: recipientUserGroups,
            itemsByChange: dropoffItemsByChange,
            earliestStartTime: dropoffEarliestStartTime,),
      ],
      isMonolithic: false,
      earliestStartTime: pickupEarliestStartTime,
      priority: 0,
    );
  }

  factory Task.dropoff({
    required String robotName,
    required String targetID,
    required Map<String, int> itemsByChange,
    required List<String> recipientUserIDs,
    required List<String> recipientUserGroups,
    required SubmoduleAddress submoduleAddress,
    required int earliestStartTime,
  }) {
    final taskID = const Uuid().v4();
    return Task(
      id: taskID,
      name: 'dropoff',
      status: 'pending',
      assigneeName: robotName,
      requirements: {},
      subtasks: [
        SubTask.assignedSubmoduleProcess(
            id: const Uuid().v4(),
            parentID: taskID,
            requiresTaskID: null,
            targetID: targetID,
            requiredUserIDs: recipientUserIDs,
            requiredUserGroups: recipientUserGroups,
            itemsByChange: itemsByChange,
            submoduleAddress: submoduleAddress,
            assigneeName: robotName,
            earliestStartTime: earliestStartTime,),
      ],
      isMonolithic: false,
      earliestStartTime: earliestStartTime,
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
