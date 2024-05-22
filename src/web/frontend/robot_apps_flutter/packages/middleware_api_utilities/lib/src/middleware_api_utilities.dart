import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:shared_data_models/shared_data_models.dart';

/// Helper class to access the middleware API.
class MiddlewareApiUtilities {
  const MiddlewareApiUtilities({required this.prefix});

  final String prefix;

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
      final response = await http.get(Uri.parse('$prefix/fleet'));
      if (response.statusCode == 200) {
        final jsonData = (jsonDecode(response.body) as Map<String, dynamic>)['fleet'] as List<dynamic>;
        for (final robot in jsonData) {
          final posReponse = await http.get(Uri.parse('$prefix/robot_pos?robot_name=$robot'));
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

  Future<List<DrawerModule>> getModules({required String robotName}) async {
    try {
      final response = await http.get(Uri.parse('$prefix/modules?robot_name=$robotName'));
      final modules = <DrawerModule>[];
      if (response.statusCode == 200) {
        final jsonData = jsonDecode(response.body) as List<dynamic>;

        for (final module in jsonData) {
          modules.add(DrawerModule.fromJson(data: module as Map<String, dynamic>));
        }
      }
      return modules;
    } catch (e) {
      return [];
    }
  }

  Future<bool> openDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/open_drawer?robot_name=$robotName&module_id=$moduleID&drawer_id=$drawerID'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> closeDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/close_drawer?robot_name=$robotName&module_id=$moduleID&drawer_id=$drawerID'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> moveRobot({
    required String robotName,
    required double x,
    required double y,
    required double yaw,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/goal_pose?robot_name=$robotName&x=$x&y=$y&z=$yaw'),
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
        Uri.parse('$prefix/block_navigation?robot_name=$robotName'),
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
        Uri.parse('$prefix/unblock_navigation?robot_name=$robotName'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> isRobotNavigationBlocked({required String robotName}) async {
    try {
      final response = await http.get(
        Uri.parse('$prefix/is_navigation_blocked?robot_name=$robotName'),
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
        Uri.parse('$prefix/is_navigation_blocked?robot_name=$robotName'),
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

  Future<bool> relabelDrawer({
    required int moduleID,
    required int drawerID,
    required String label,
    required String robotName,
  }) async {
    final data = <String, dynamic>{
      'module_id': moduleID,
      'drawer_id': drawerID,
      'robot_name': robotName,
      'label': label,
      'status': 'Closed',
    };
    final headers = {
      'accept': 'application/json',
      'Content-Type': 'application/json',
    };
    try {
      final response = await http.post(
        Uri.parse('$prefix/robots/modules/status'),
        headers: headers,
        body: jsonEncode(data),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }
}
