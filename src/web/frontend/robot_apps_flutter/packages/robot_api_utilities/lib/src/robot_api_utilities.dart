import 'dart:convert';

import 'package:shared_data_models/shared_data_models.dart';
import 'package:http/http.dart' as http;

/// {@template robot_api_utilities}
/// A Very Good Project created by Very Good CLI.
/// {@endtemplate}
class RobotApiUtilities {
  /// {@macro robot_api_utilities}
  const RobotApiUtilities({required this.prefix});

  final String prefix;

  Future<Pose?> getRobotPose() async {
    try {
      final posReponse = await http.get(Uri.parse('$prefix/robot_pos'));
      final posData = jsonDecode(posReponse.body) as Map<String, dynamic>;
      final pose = Pose(
        x: posData['x'] as double,
        y: posData['y'] as double,
        yaw: posData['z'] as double,
      );
      return pose;
    } catch (e) {
      return null;
    }
  }

  Future<List<DrawerModule>?> getModules() async {
    final modules = <DrawerModule>[];
    try {
      final response = await http.get(Uri.parse('$prefix/modules'));
      if (response.statusCode == 200) {
        final jsonData = jsonDecode(response.body) as List<dynamic>;
        for (final module in jsonData) {
          modules.add(DrawerModule.fromJson(data: module as Map<String, dynamic>));
        }
      }
      return modules;
    } catch (e) {
      return null;
    }
  }

  Future<bool> openDrawer({
    required int moduleID,
    required int drawerID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/open_drawer?module_id=$moduleID&drawer_id=$drawerID'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> closeDrawer({
    required int moduleID,
    required int drawerID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/close_drawer?module_id=$moduleID&drawer_id=$drawerID'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> blockNavigation() async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/block_navigation'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> unblockNavigation() async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/unblock_navigation'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> isNavigationBlocked() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/is_navigation_blocked'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        return data['is_nav_blocked'] as bool;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  Future<ModuleProcess?> getModuleProcess() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/module_process_status'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        return ModuleProcess.fromJson(data["success"]);
      } else {
        return null;
      }
    } catch (e) {
      return null;
    }
  }

  Future<bool> finishModuleProcess() async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/finish_module_process'),
      );
      if (response.statusCode == 200) {
        return true;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }
}
