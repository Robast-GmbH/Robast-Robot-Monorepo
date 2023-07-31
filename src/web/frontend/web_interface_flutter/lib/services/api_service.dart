import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot.dart';
import 'package:web_interface_flutter/models/task.dart';
import 'package:web_interface_flutter/models/user.dart';

class APIService {
  static const baseURL = "http://localhost";
  static const port = 3001;
  static Future<int> postTask({
    required int ownerID,
    required int targetID,
    required int moduleID,
    required int drawerID,
    required double xPose,
    required double yPose,
    required double yawPose,
  }) async {
    final data = <String, dynamic>{
      "owner_id": ownerID,
      "target_id": targetID,
      "module_id": moduleID,
      "drawer_id": drawerID,
      "x_pose": xPose,
      "y_pose": yPose,
      "yaw_pose": yawPose
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    final response = await http.post(Uri.parse("$baseURL:$port/tasks/"), headers: headers, body: jsonEncode(data));
    return response.statusCode;
  }

  static Future<List<User>> getUsers() async {
    final users = <User>[];
    final response = await http.get(Uri.parse("$baseURL:$port/users/?skip=0&limit=100"));
    if (response.statusCode == 200) {
      final List<dynamic> jsonData = jsonDecode(response.body);
      for (final user in jsonData) {
        users.add(User.fromJson(user));
      }
    }
    return users;
  }

  static Future<List<Robot>> getRobots() async {
    final robots = <Robot>[];
    final response = await http.get(Uri.parse("$baseURL:$port/robots"));
    if (response.statusCode == 200) {
      final List<dynamic> jsonData = jsonDecode(response.body);
      for (final robot in jsonData) {
        robots.add(Robot.fromJson(robot));
      }
    }
    return robots;
  }

  static Future<List<Task>> getTasks() async {
    final tasks = <Task>[];
    final response = await http.get(Uri.parse("$baseURL:$port/tasks"));
    if (response.statusCode == 200) {
      final List<dynamic> jsonData = jsonDecode(response.body);
      for (final task in jsonData) {
        tasks.add(Task.fromJson(task));
      }
    }
    return tasks;
  }

  static Future<List<DrawerModule>> getModules(String robotName) async {
    final modules = <DrawerModule>[];
    final response = await http.get(Uri.parse("$baseURL:$port/robots/$robotName/modules/"));
    if (response.statusCode == 200) {
      final List<dynamic> jsonData = jsonDecode(response.body);
      for (final moduleID in jsonData) {
        modules.add(DrawerModule(moduleID: moduleID));
      }
    }
    return modules;
  }

  static Future<void> openDrawer(String robotName, int moduleID) async {}

  static Future<void> closeDrawer(String robotName, int moduleID) async {}
}
