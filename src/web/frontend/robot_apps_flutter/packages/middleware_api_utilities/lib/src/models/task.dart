class Action {
  Action({
    required this.name,
    required this.status,
    required this.parameters,
    required this.subaction,
  });

  factory Action.fromJson(Map<String, dynamic> json) {
    return Action(
      name: json['name'] as String,
      status: json['status'] as String,
      parameters: json['parameters'] as Map<String, dynamic>,
      subaction: json['subaction'] != null ? Action.fromJson(json['subaction'] as Map<String, dynamic>) : null,
    );
  }

  final String name;
  final String status;
  final Map<String, dynamic> parameters;
  final Action? subaction;

  Map<String, dynamic> toJson() {
    return {
      'name': name,
      'status': status,
      'parameters': parameters,
      'subaction': subaction?.toJson(),
    };
  }
}

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
        name: 'drawer_process',
        status: 'pending',
        parameters: {
          'items_by_change': itemsByChange,
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

class Task {
  Task({
    required this.id,
    required this.name,
    required this.status,
    required this.assigneeName,
    required this.requirements,
    required this.subtasks,
    required this.isMonolithic,
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
    );
  }

  final String id;
  final String name;
  final String status;
  final String assigneeName;
  final Map<String, dynamic> requirements;
  final List<SubTask> subtasks;
  final bool isMonolithic;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'status': status,
      'assignee_name': assigneeName,
      'requirements': requirements,
      'subtasks': subtasks.map((subtask) => subtask.toJson()).toList(),
      'is_monolithic': isMonolithic,
    };
  }
}
