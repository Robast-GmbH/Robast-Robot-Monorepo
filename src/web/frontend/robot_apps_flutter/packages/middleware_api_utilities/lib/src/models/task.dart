import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class Task {
  Task({
    required this.id,
    required this.taskType,
    required this.status,
    required this.payload,
    required this.creationDate,
    required this.requiresTaskID,
    required this.assigneeName,
    required this.address,
    required this.drawerType,
    required this.priority,
    required this.targetID,
    required this.authUsers,
    required this.authUserGroups,
    required this.taskPhases,
  });

  factory Task.fromJson(Map<String, dynamic> json) {
    return Task(
      id: json['id'] as String,
      taskType: json['task_type'] as String,
      status: json['status'] as String,
      payload: (json['payload'] as Map<String, dynamic>).map((key, value) => MapEntry(key, value as int)),
      creationDate: json['creation_date'] as int,
      requiresTaskID: json['requires_task_id'] as String?,
      assigneeName: json['assignee_name'] as String,
      address: DrawerAddress(
        moduleID: json['module_id'] as int,
        drawerID: json['drawer_id'] as int,
      ),
      drawerType: json['drawer_type'] as int,
      priority: json['priority'] as int,
      targetID: json['target_id'] as String,
      authUsers: (json['auth_users'] as List).map((e) => e as String).toList(),
      authUserGroups: (json['auth_user_groups'] as List).map((e) => e as String).toList(),
      taskPhases: (json['task_phases'] as List)
          .map(
            (e) => ((e as Map<String, dynamic>)['activity'] as Map<String, dynamic>)['category'] as String,
          )
          .toList(),
    );
  }

  final String id;
  final String taskType;
  final String status;
  final Map<String, int> payload;
  final int creationDate;
  final String? requiresTaskID;
  final String assigneeName;
  final DrawerAddress address;
  final int drawerType;
  final int priority;
  final String targetID;
  final List<String> authUsers;
  final List<String> authUserGroups;
  final List<String> taskPhases;
}
