import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:shared_data_models/shared_data_models.dart';

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

  Future<bool> openSubmodule({
    required int moduleID,
    required int submoduleID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/open_submodule?module_id=$moduleID&submodule_id=$submoduleID'),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<bool> closeSubmodule({
    required int moduleID,
    required int submoduleID,
  }) async {
    try {
      final response = await http.post(
        Uri.parse('$prefix/close_submodule?module_id=$moduleID&submodule_id=$submoduleID'),
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

  Future<bool> waitForDisinfectionTriggered({int timeout = 10}) async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/disinfection_triggered?timeout=$timeout'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        return data['status'] as String == 'success';
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  Future<bool> getIsRobotLost() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/robot_lost'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        return data['data'] as bool;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  Future<bool> setInitialRobotPoint({required Pose pose}) async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.post(
        Uri.parse('$prefix/set_initial_point?x=${pose.x}&y=${pose.y}&z=${pose.yaw}'),
        headers: headers,
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

  Future<BatteryStatus?> getBatteryStatus() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/battery_status'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        if (data['status'] == 'success') {
          return BatteryStatus.fromJson(data['data'] as Map<String, dynamic>);
        }
      }
      return null;
    } catch (e) {
      return null;
    }
  }

  Future<int?> getRemainingDisinfections() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/disinfection_module_status'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        if (data['status'] == 'success') {
          return data['remaining_disinfections'] as int;
        }
      }
      return null;
    } catch (e) {
      return null;
    }
  }

  Future<bool> refillDisinfectionFluidContainer() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.post(
        Uri.parse('$prefix/refill_disinfection_fluid_container'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        return true;
      }
      return false;
    } catch (e) {
      return false;
    }
  }

  Future<bool?> getEmergencyStopPressed() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/emergency_stop_pressed'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as Map<String, dynamic>;
        return data['data'] as bool;
      }
      return null;
    } catch (e) {
      return null;
    }
  }

  Future<Set<int>?> getHeartbeatTimeoutErrors() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/heartbeat_timeout_errors'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as List<dynamic>;
        return data
            .map(
              (e) => int.parse(
                (e as Map<String, dynamic>)['error_data'] as String,
              ),
            )
            .toSet();
      }
      return null;
    } catch (e) {
      return null;
    }
  }

  Future<Set<int>?> getLivingDevices() async {
    try {
      final headers = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/living_devices'),
        headers: headers,
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body) as List<dynamic>;
        return data.map((e) => e as int).toSet();
      }
      return null;
    } catch (e) {
      return null;
    }
  }
}
