import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_interface_flutter/models/data/drawer_module.dart';
import 'package:web_interface_flutter/models/data/robot.dart';
import 'package:web_interface_flutter/models/data/task.dart';
import 'package:web_interface_flutter/models/data/user.dart';

class APIService {
  static String baseURL = "http://10.10.23.7"; //"http://localhost"; // "http://10.0.2.2"; //"http://10.10.23.10";
  static int port = 3000;

  static Future<bool> testConnection({
    required String url,
  }) async {
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
            "step": 1,
            "type": "DRAWER",
            "action": {
              "drawer_id": drawerID,
              "module_id": moduleID,
              "locked_for": [targetUser],
            },
          },
          {
            "step": 2,
            "type": "NAVIGATION",
            "action": {
              "pose": {"x": xPose, "y": yPose, "z": 0},
              "yaw": yawPose,
            },
          },
          {
            "step": 3,
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

  static Future<Task?> getTaskByID(int taskID) async {
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/tasks/$taskID"));
      if (response.statusCode == 200) {
        final Map<String, dynamic> jsonData = jsonDecode(response.body);

        return Task.fromJson(jsonData);
      }
    } catch (e) {
      debugPrint("Failed get Tasks");
      debugPrint(e.toString());
    }
    return null;
  }

  static Future<List<Task>> getTasks() async {
    final tasks = <Task>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/tasks"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body);
        for (final taskData in jsonData) {
          final task = Task.fromJson(taskData);
          tasks.add(task);
        }
      }
    } catch (e) {
      debugPrint("Failed get Tasks");
      debugPrint(e.toString());
    }

    return tasks;
  }

  static Future<List<DrawerModule>> getModules({required String robotName}) async {
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

  static Future<void> openDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
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

  static Future<void> closeDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
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

  static Future<void> moveRobot({
    required String robotName,
    required double x,
    required double y,
    required double yaw,
    required int ownerID,
  }) async {
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
        body: jsonEncode(data),
      );
    } catch (e) {
      debugPrint("Robot move failed");
      debugPrint(e.toString());
    }
  }

  static Future<void> relabelDrawer({
    required int moduleID,
    required int drawerID,
    required String label,
    required String robotName,
  }) async {
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

  static Future<void> resetModules({required String robotName}) async {
    try {
      await http.post(
        Uri.parse("$baseURL:$port/robots/$robotName/modules/reset"),
      );
    } catch (e) {
      debugPrint("Failed to reset modules");
      debugPrint(e.toString());
    }
  }

  static Future<bool> startPatrol({
    required String fleetName,
    required String robotName,
    required List<Offset> points,
    required List<double> yaws,
    required int ownerID,
    required String taskID,
  }) async {
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    final actions = List.generate(
        points.length,
        (index) => {
              "step": index + 1,
              "type": "NAVIGATION",
              "action": {
                "pose": {
                  "x": points[index].dx,
                  "y": points[index].dy,
                  "z": 0,
                },
                "yaw": yaws[index],
              },
              "finished": false
            });
    final data = <String, dynamic>{
      "task_id": taskID,
      "robot": {
        "fleet_name": fleetName,
        "robot_name": robotName,
      },
      "actions": actions,
    };
    try {
      await http.put(
        Uri.parse("$baseURL:$port/robots/loop"),
        headers: headers,
        body: jsonEncode(data),
      );
      return true;
    } catch (e) {
      debugPrint("Robot move failed");
      debugPrint(e.toString());
      return false;
    }
  }

  static Future<void> pauseOrResumeRobot({
    required String fleetName,
    required String robotName,
    required bool shouldPause,
  }) async {
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };

    try {
      await http.put(
        Uri.parse("$baseURL:$port/robots/pause_resume/?robot_name=$robotName&fleet_name=$fleetName&pause=$shouldPause"),
        headers: headers,
      );
    } catch (e) {
      debugPrint("Robot ${shouldPause ? "pause" : "resume"} failed");
      debugPrint(e.toString());
    }
  }
}
