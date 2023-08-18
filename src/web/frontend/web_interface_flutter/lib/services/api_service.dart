import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot.dart';
import 'package:web_interface_flutter/models/task.dart';
import 'package:web_interface_flutter/models/user.dart';

class APIService {
  static String baseURL = "http://localhost"; //"http://10.10.23.10";
  static int port = 3001;

  static Future<bool> testConnection(String url) async {
    try {
      final response = await http.get(Uri.parse("$url/"));
      if (response.statusCode == 200) {
        return true;
      }
    } catch (e) {
      debugPrint(e.toString());
    }
    return false;
  }

  static Future<int> postTask({
    required String taskID,
    required int ownerID,
    required String targetUser,
    required int moduleID,
    required int drawerID,
    required double xPose,
    required double yPose,
    required double yawPose,
    required String fleetName,
    required String robotName,
  }) async {
    final data = <String, dynamic>{
      "task": {
        "task_id": taskID,
        "actions": [
          {
            "step": 0,
            "type": "DRAWER",
            "action": {
              "drawer_id": drawerID,
              "module_id": moduleID,
              "locked_for": [targetUser],
            },
          },
          {
            "step": 1,
            "type": "NAVIGATION",
            "action": {
              "pose": {"x": xPose, "y": yPose, "z": 0},
              "yaw": yawPose,
            },
          },
          {
            "step": 2,
            "type": "DRAWER",
            "action": {
              "drawer_id": drawerID,
              "module_id": moduleID,
              "locked_for": [],
            },
          },
        ],
      },
      "robot": {
        "fleet_name": fleetName,
        "robot_name": robotName,
      },
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    final response = await http.post(Uri.parse("$baseURL:$port/tasks/?user_id=$ownerID"), headers: headers, body: jsonEncode(data));
    return response.statusCode;
  }

  static Future<List<User>> getUsers() async {
    final users = <User>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/users/?skip=0&limit=100"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body);
        for (final user in jsonData) {
          users.add(User.fromJson(user));
        }
      }
    } catch (e) {
      debugPrint("Failed get Users");
      debugPrint(e.toString());
    }

    return users;
  }

  static Future<List<Robot>> getRobots() async {
    final robots = <Robot>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/robots"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body);
        for (final robot in jsonData) {
          robots.add(Robot.fromJson(robot));
        }
      }
    } catch (e) {
      debugPrint("Failed get Robots");
      debugPrint(e.toString());
    }
    return robots;
  }

  static Future<List<Task>> getTasks() async {
    final tasks = <Task>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/tasks"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body);
        for (final task in jsonData) {
          tasks.add(Task.fromJson(task));
        }
      }
    } catch (e) {
      debugPrint("Failed get Tasks");
      debugPrint(e.toString());
    }

    return tasks;
  }

  static Future<List<DrawerModule>> getModules(String robotName) async {
    final modules = <DrawerModule>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/robots/$robotName/modules/"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body);
        for (final module in jsonData) {
          modules.add(DrawerModule.fromJson(module));
        }
      }
    } catch (e) {
      debugPrint("Failed get Modules");
      debugPrint(e.toString());
    }

    return modules;
  }

  static Future<void> openDrawer(String robotName, int moduleID, int drawerID) async {
    final data = <String, dynamic>{
      "module": {
        "module_id": moduleID,
        "drawer_id": drawerID,
      },
      "owner": 1,
      "restricted_for_user": [],
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      await http.post(
        Uri.parse("$baseURL:$port/robots/$robotName/modules/open"),
        headers: headers,
        body: jsonEncode(data),
      );
    } catch (e) {
      debugPrint("Failed open drawer");
      debugPrint(e.toString());
    }
  }

  static Future<void> closeDrawer(String robotName, int moduleID, int drawerID) async {
    final data = <String, dynamic>{
      "module_id": moduleID,
      "drawer_id": drawerID,
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      await http.post(
        Uri.parse("$baseURL:$port/robots/$robotName/modules/close"),
        headers: headers,
        body: jsonEncode(data),
      );
    } catch (e) {
      debugPrint("Failed close drawer");
      debugPrint(e.toString());
    }
  }

  static Future<void> moveRobot(String robotName, double x, double y, double yaw, int ownerID) async {
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };

    final data = {
      "target": {
        "pose": {
          "x": x,
          "y": y,
          "z": 0,
        },
        "yaw": yaw,
      },
      "owner_id": ownerID
    };
    try {
      await http.put(
        Uri.parse("$baseURL:$port/robots/$robotName/navigate"),
        headers: headers,
        body: data,
      );
    } catch (e) {
      debugPrint("Robot move failed");
      debugPrint(e.toString());
    }
  }

  static Future<void> relabelDrawer(int moduleID, int drawerID, String label, String robotName) async {
    final data = <String, dynamic>{
      "module_id": moduleID,
      "drawer_id": drawerID,
      "robot_name": robotName,
      "label": label,
      "status": "Closed",
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      await http.post(
        Uri.parse("$baseURL:$port/robots/modules/status"),
        headers: headers,
        body: jsonEncode(data),
      );
    } catch (e) {
      debugPrint("Failed to relabel drawer");
      debugPrint(e.toString());
    }
  }
}
