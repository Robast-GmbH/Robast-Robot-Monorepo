class Drawer {
  final String robotName;
  final int moduleId;
  final int drawerId;
  final int size;

  final Map<String, int> content;
  final List<String> reservedForIds;
  final List<String> reservedForGroups;
  final String moduleProcessStatus;
  final String moduleProcessType;
  final Map<String, int> moduleProcessPayload;
  Drawer({
    required this.robotName,
    required this.moduleId,
    required this.drawerId,
    required this.size,
    required this.content,
    required this.reservedForIds,
    required this.reservedForGroups,
    required this.moduleProcessStatus,
    required this.moduleProcessType,
    required this.moduleProcessPayload,
  });

  // Factory method to create a Drawer instance from a JSON object
  factory Drawer.fromJson(Map<String, dynamic> json) {
    return Drawer(
      robotName: json['robot_name'],
      moduleId: json['module_id'],
      drawerId: json['drawer_id'],
      size: json['size'],
      content: Map<String, int>.from(json['content']),
      reservedForIds: List<String>.from(json['reserved_for_ids']),
      reservedForGroups: List<String>.from(json['reserved_for_groups']),
      moduleProcessStatus: json['module_process_status'],
      moduleProcessType: json['module_process_type'],
      moduleProcessPayload: Map<String, int>.from(json['module_process_payload']),
    );
  }

  // Method to convert a Drawer instance to a JSON object
  Map<String, dynamic> toJson() {
    return {
      'robot_name': robotName,
      'module_id': moduleId,
      'drawer_id': drawerId,
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
