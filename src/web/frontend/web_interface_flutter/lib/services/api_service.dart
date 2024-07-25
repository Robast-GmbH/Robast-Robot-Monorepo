import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_interface_flutter/models/data/drawer_module.dart';
import 'package:web_interface_flutter/models/data/robot.dart';
import 'package:web_interface_flutter/models/data/task.dart';
import 'package:web_interface_flutter/models/data/user.dart';

class APIService {
  static String baseURL = "http://localhost"; // "http://10.0.2.2"; //"http://10.10.23.10";//"http://10.10.23.7"; //
  static int port = 8003;

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




  static Future<List<Robot>> getRobots() async {
    final robots = <Robot>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/fleet"));
      if (response.statusCode == 200) {
        final List<dynamic> jsonData = jsonDecode(response.body)["fleet"];
        for (final robot in jsonData) {
          final posReponse = await http.get(Uri.parse("$baseURL:$port/robot_pos?robot_name=$robot"));
          final Map<String,dynamic> posData = jsonDecode(posReponse.body);
          final robotData = {
            "robot_name": robot,
            "fleet_name": "Flotte 0",
            "x_pose": posData["x"],
            "y_pose":posData["y"],
            "yaw_pose": posData["z"],
            "task_id": 0,
            "battery_level": 75.0,
          };
          robots.add(Robot.fromJson(robotData));
        }
      }
    } catch (e) {
      debugPrint("Failed get Robots");
      debugPrint(e.toString());
    }
    return robots;
  }

 
  static Future<List<DrawerModule>> getModules({required String robotName}) async {
    final modules = <DrawerModule>[];
    try {
      final response = await http.get(Uri.parse("$baseURL:$port/modules?robot_name=$robotName"));
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
    try {
      await http.post(
        Uri.parse("$baseURL:$port/open_drawer?robot_name=$robotName&module_id=$moduleID&drawer_id=$drawerID"),
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

    try {
      await http.post(
        Uri.parse("$baseURL:$port/close_drawer?robot_name=$robotName&module_id=$moduleID&drawer_id=$drawerID"),
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

    try {
      await http.post(
        Uri.parse("$baseURL:$port/goal_pose?robot_name=$robotName&x=$x&y=$y&z=$yaw"),
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
