import 'package:web_interface_flutter/models/task.dart';

class User {
  final int id;
  final String name;
  final String fullName;
  final bool isActive;
  final bool isAdmin;
  final List<Task> tasks;

  User({
    required this.tasks,
    required this.id,
    required this.name,
    required this.fullName,
    required this.isActive,
    required this.isAdmin,
  });

  static User fromJson(Map<String, dynamic> data) {
    return User(
        fullName: data["full_name"],
        name: data["name"],
        id: data["id"],
        isActive: data["is_active"],
        isAdmin: data["admin"],
        tasks: (data["orders"] as List<dynamic>).map((e) => Task.fromJson(e)).toList());
  }
}
