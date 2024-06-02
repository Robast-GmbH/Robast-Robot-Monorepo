class Task {
  Task.fromJson(Map<String, dynamic> json)
      : id = (json['booking'] as Map<String, dynamic>)['id'] as String,
        assignee = ((json['assigned_to'] as Map<String, dynamic>?) ?? {'name': 'not assigned'})['name'] as String,
        category = json['category'] as String,
        state = json['status'] as String;
  final String id;
  final String assignee;
  final String category;
  final String state;
}
