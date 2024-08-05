import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';
import 'package:middleware_api_utilities/src/sub_apis/modules_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/nfc_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/tasks_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/users_api.dart';
import 'package:shared_data_models/shared_data_models.dart';

/// Helper class to access the middleware API.
class MiddlewareApiUtilities {
  factory MiddlewareApiUtilities() {
    return _instance;
  }
  MiddlewareApiUtilities._internal();

  static final _instance = MiddlewareApiUtilities._internal();

  void setPrefix({required String prefix}) {
    _prefix = prefix;
    _modules = ModulesApi(prefix: prefix);
    _users = UsersApi(prefix: prefix);
    _tasks = TasksApi(prefix: prefix);
    _nfc = NFCApi(prefix: prefix);
  }

  late String _prefix;
  late ModulesApi _modules;
  late UsersApi _users;
  late TasksApi _tasks;
  late NFCApi _nfc;

  ModulesApi get modules => _modules;
  UsersApi get users => _users;
  TasksApi get tasks => _tasks;
  NFCApi get nfc => _nfc;

  Future<bool> testConnection({
    required String url,
  }) async {
    try {
      final response = await http.get(Uri.parse('$url/'));
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<List<Robot>> getRobots() async {
    final robots = <Robot>[];
    try {
      final response = await http.get(Uri.parse('$_prefix/fleet'));
      if (response.statusCode == 200) {
        final jsonData = (jsonDecode(response.body) as Map<String, dynamic>)['fleet'] as List<dynamic>;
        for (final robot in jsonData) {
          final posReponse = await http.get(Uri.parse('$_prefix/robot_pos?robot_name=$robot'));
          final posData = jsonDecode(posReponse.body) as Map<String, dynamic>;
          final robotData = {
            'robot_name': robot,
            'fleet_name': 'Flotte 0',
            'x_pose': posData['x'],
            'y_pose': posData['y'],
            'yaw_pose': posData['z'],
            'battery_level': 75.0,
          };
          robots.add(Robot.fromJson(data: robotData));
        }
      }
    } catch (e) {
      return [];
    }
    return robots;
  }

  Future<bool> moveRobot({
    required String robotName,
    required double x,
    required double y,
    required double yaw,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$_prefix/goal_pose?robot_name=$robotName&x=$x&y=$y&z=$yaw'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> stopRobot({
    required String robotName,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$_prefix/block_navigation?robot_name=$robotName'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> resumeRobot({
    required String robotName,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$_prefix/unblock_navigation?robot_name=$robotName'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> isRobotNavigationBlocked({required String robotName}) async {
    try {
      final response = await http.get(
        Uri.parse('$_prefix/is_navigation_blocked?robot_name=$robotName'),
      );
      if (response.statusCode == 200) {
        return (jsonDecode(response.body) as Map<String, dynamic>)['is_nav_blocked'] as bool;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  Future<bool> isNavigationBlocked({required String robotName}) async {
    try {
      final response = await http.get(
        Uri.parse('$_prefix/is_navigation_blocked?robot_name=$robotName'),
      );
      if (response.statusCode == 200) {
        return (jsonDecode(response.body) as Map<String, dynamic>)['is_nav_blocked'] as bool;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  Future<BuildingMap?> getMap() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$_prefix/building_map'));
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      return BuildingMap.fromJson(data);
    } else {
      return null;
    }
  }
}
