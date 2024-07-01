class RobotDrawer {
  RobotDrawer({
    required this.robotName,
    required this.moduleID,
    required this.drawerID,
    required this.position,
    required this.size,
    required this.content,
    required this.reservedForIds,
    required this.reservedForGroups,
    required this.moduleProcessStatus,
    required this.moduleProcessType,
    required this.moduleProcessPayload,
  });

  // Factory method to create a Drawer instance from a JSON object
  factory RobotDrawer.fromJson(Map<String, dynamic> json) {
    return RobotDrawer(
      robotName: json['robot_name'] as String,
      moduleID: json['module_id'] as int,
      drawerID: json['drawer_id'] as int,
      position: json['position'] as int,
      size: json['size'] as int,
      content: Map<String, int>.from(json['content'] as Map<String, dynamic>),
      reservedForIds: List<String>.from(json['reserved_for_ids'] as List<dynamic>),
      reservedForGroups: List<String>.from(json['reserved_for_groups'] as List<dynamic>),
      moduleProcessStatus: json['module_process_status'] as String,
      moduleProcessType: json['module_process_type'] as String,
      moduleProcessPayload: Map<String, int>.from(json['module_process_payload'] as Map<String, dynamic>),
    );
  }
  final String robotName;
  final int moduleID;
  final int drawerID;
  final int position;
  final int size;

  final Map<String, int> content;
  final List<String> reservedForIds;
  final List<String> reservedForGroups;
  final String moduleProcessStatus;
  final String moduleProcessType;
  final Map<String, int> moduleProcessPayload;

  // Method to convert a Drawer instance to a JSON object
  Map<String, dynamic> toJson() {
    return {
      'robot_name': robotName,
      'module_id': moduleID,
      'drawer_id': drawerID,
      'position': position,
      'size': size,
      'content': content,
      'reserved_for_ids': reservedForIds,
      'reserved_for_groups': reservedForGroups,
      'module_process_status': moduleProcessStatus,
      'module_process_type': moduleProcessType,
      'module_process_payload': moduleProcessPayload,
    };
  }
}
