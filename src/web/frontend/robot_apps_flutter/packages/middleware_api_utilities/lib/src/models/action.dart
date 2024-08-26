class Action {
  Action({
    required this.id,
    required this.name,
    required this.status,
    required this.parameters,
    required this.subaction,
  });

  factory Action.fromJson(Map<String, dynamic> json) {
    return Action(
      id: json['id'] as String,
      name: json['name'] as String,
      status: json['status'] as String,
      parameters: json['parameters'] as Map<String, dynamic>,
      subaction: json['subaction'] != null ? Action.fromJson(json['subaction'] as Map<String, dynamic>) : null,
    );
  }

  final String id;
  final String name;
  final String status;
  final Map<String, dynamic> parameters;
  final Action? subaction;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'name': name,
      'status': status,
      'parameters': parameters,
      'subaction': subaction?.toJson(),
    };
  }
}
