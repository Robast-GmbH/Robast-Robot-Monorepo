class Task {
  Task({
    required this.id,
    required this.assigneeName,
    required this.taskType,
    required this.requiresTaskID,
    required this.moduleID,
    required this.drawerID,
    required this.targetPlace,
  });

  factory Task.fromJson(Map<String, dynamic> json) {
    return Task(
      id: json['id'] as String,
      assigneeName: json['assigneeName'] as String,
      taskType: json['taskType'] as String,
      requiresTaskID: json['requiresTaskID'] as String,
      moduleID: json['moduleID'] as int,
      drawerID: json['drawerID'] as int,
      targetPlace: json['targetPlace'] as String,
    );
  }
  final String id;
  final String assigneeName;
  final String taskType;
  final String requiresTaskID;
  final int moduleID;
  final int drawerID;
  final String targetPlace;
}
