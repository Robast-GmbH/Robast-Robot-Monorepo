import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:middleware_api_utilities/src/models/action.dart';
import 'package:middleware_api_utilities/src/models/drawer_address.dart';
import 'package:uuid/uuid.dart';

class SubTask {
  SubTask({
    required this.id,
    required this.name,
    required this.status,
    required this.assigneeName,
    required this.parentID,
    required this.requiresTaskID,
    required this.isPartOfMonolith,
    required this.targetID,
    required this.priority,
    required this.earliestStartTime,
    required this.requirements,
    required this.action,
  });

  factory SubTask.fromJson(Map<String, dynamic> json) {
    return SubTask(
      id: json['id'] as String,
      name: json['name'] as String,
      status: json['status'] as String,
      assigneeName: json['assignee_name'] as String,
      parentID: json['parent_id'] as String,
      requiresTaskID: json['requires_task_id'] as String?,
      isPartOfMonolith: json['is_part_of_monolith'] as bool,
      targetID: json['target_id'] as String,
      priority: json['priority'] as int,
      earliestStartTime: json['earliest_start_time'] as int,
      requirements: json['requirements'] as Map<String, dynamic>,
      action: Action.fromJson(json['action'] as Map<String, dynamic>),
    );
  }

  factory SubTask.drawerProcess({
    required String id,
    required String parentID,
    required String? requiresTaskID,
    required String targetID,
    required List<String> requiredUserIDs,
    required List<String> requiredUserGroups,
    required Map<String, int> itemsByChange,
  }) {
    return SubTask(
      id: id,
      name: 'drawer_process',
      status: 'pending',
      assigneeName: '',
      requiresTaskID: requiresTaskID,
      isPartOfMonolith: false,
      parentID: parentID,
      targetID: targetID,
      priority: 0,
      earliestStartTime: DateTime.now().millisecondsSinceEpoch ~/ 1000,
      requirements: {
        'required_user_ids': requiredUserIDs,
        'required_user_groups': requiredUserGroups,
      },
      action: Action(
        id: const Uuid().v4(),
        name: 'drawer_process',
        status: 'pending',
        parameters: {
          'items_by_change': itemsByChange,
        },
        subaction: null,
      ),
    );
  }

  factory SubTask.assignedDrawerProcess({
    required String id,
    required String parentID,
    required String? requiresTaskID,
    required String targetID,
    required List<String> requiredUserIDs,
    required List<String> requiredUserGroups,
    required Map<String, int> itemsByChange,
    required DrawerAddress drawerAddress,
    required String assigneeName,
  }) {
    final completeDrawerAddress = {'drawer_address': drawerAddress.toJson()};
    completeDrawerAddress['drawer_address']!['robot_name'] = assigneeName;
    return SubTask(
      id: id,
      name: 'drawer_process',
      status: 'pending',
      assigneeName: assigneeName,
      requiresTaskID: requiresTaskID,
      isPartOfMonolith: false,
      parentID: parentID,
      targetID: targetID,
      priority: 0,
      earliestStartTime: DateTime.now().millisecondsSinceEpoch ~/ 1000,
      requirements: {
        'required_user_ids': requiredUserIDs,
        'required_user_groups': requiredUserGroups,
        ...completeDrawerAddress,
      },
      action: Action(
        id: const Uuid().v4(),
        name: 'drawer_process',
        status: 'pending',
        parameters: {
          'items_by_change': itemsByChange,
          ...completeDrawerAddress,
        },
        subaction: null,
      ),
    );
  }

  final String id;
  final String name;
  final String status;
  final String assigneeName;
  final String parentID;
  final String? requiresTaskID;
  final bool isPartOfMonolith;
  final String targetID;
  final int priority;
  final int earliestStartTime;
  final Map<String, dynamic> requirements;
  final Action action;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'status': status,
      'assignee_name': assigneeName,
      'parent_id': parentID,
      'requires_task_id': requiresTaskID,
      'is_part_of_monolith': isPartOfMonolith,
      'target_id': targetID,
      'priority': priority,
      'earliest_start_time': earliestStartTime,
      'requirements': requirements,
      'action': action.toJson(),
    };
  }
}
