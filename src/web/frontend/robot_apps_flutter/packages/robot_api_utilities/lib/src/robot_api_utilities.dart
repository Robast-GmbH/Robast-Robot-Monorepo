import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:shared_data_models/shared_data_models.dart';

/// {@template robot_api_utilities}
/// A Very Good Project created by Very Good CLI.
/// {@endtemplate}
class RobotApiUtilities {
  /// {@macro robot_api_utilities}
  const RobotApiUtilities({required this.prefix});

  static const _heartbeatErrorCode = '50304';

  final String prefix;

  Future<Pose?> getRobotPose() async => _getRequest(
        endpoint: 'robot_pos',
        parser: (body) {
          final posData = jsonDecode(body) as Map<String, dynamic>;
          final pose = Pose(
            x: posData['x'] as double,
            y: posData['y'] as double,
            yaw: posData['z'] as double,
          );
          return pose;
        },
      );

  Future<bool?> openSubmodule({
    required int moduleID,
    required int submoduleID,
  }) async =>
      _postRequest(
        endpoint: 'open_submodule?module_id=$moduleID&submodule_id=$submoduleID',
        parser: (body) => true,
      );

  Future<bool?> closeSubmodule({
    required int moduleID,
    required int submoduleID,
  }) async =>
      _postRequest(
        endpoint: 'close_submodule?module_id=$moduleID&submodule_id=$submoduleID',
        parser: (body) => true,
      );

  Future<bool?> blockNavigation() async => _postRequest(
        endpoint: 'block_navigation',
        parser: (body) => true,
      );

  Future<bool?> unblockNavigation() async => _postRequest(
        endpoint: 'unblock_navigation',
        parser: (body) => true,
      );

  Future<bool?> isNavigationBlocked() async => _getRequest(
        endpoint: 'is_navigation_blocked',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          return data['is_nav_blocked'] as bool;
        },
      );

  Future<bool?> waitForDisinfectionTriggered({
    int timeout = 10,
  }) async =>
      _getRequest(
        endpoint: 'disinfection_triggered?timeout=$timeout',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          return data['status'] as String == 'success';
        },
      );

  Future<bool?> getIsRobotLost() async => _getRequest(
        endpoint: 'robot_lost',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          return data['data'] as bool;
        },
      );

  Future<bool?> setInitialRobotPoint({
    required Pose pose,
  }) async =>
      _postRequest(
        endpoint: 'set_initial_point?x=${pose.x}&y=${pose.y}&z=${pose.yaw}',
        parser: (body) => true,
      );

  Future<BatteryStatus?> getBatteryStatus() async => _getRequest(
        endpoint: 'battery_status',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          if (data['status'] == 'success') {
            return BatteryStatus.fromJson(data['data'] as Map<String, dynamic>);
          }
          return null;
        },
      );

  Future<int?> getRemainingDisinfections() async => _getRequest(
        endpoint: 'disinfection_module_status',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          if (data['status'] == 'success') {
            return data['remaining_disinfections'] as int;
          }
          return null;
        },
      );

  Future<bool?> refillDisinfectionFluidContainer() async => _postRequest(
        endpoint: 'refill_disinfection_fluid_container',
        parser: (body) {
          return true;
        },
      );

  Future<bool?> getEmergencyStopPressed() async => _getRequest(
        endpoint: 'emergency_stop_pressed',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          return data['data'] as bool;
        },
      );

  Future<Set<int>?> getHeartbeatTimeoutErrors() async => _getRequest(
        endpoint: 'errors',
        parser: (body) {
          final data = jsonDecode(body) as Map<String, dynamic>;
          if (data.containsKey(_heartbeatErrorCode)) {
            return (data[_heartbeatErrorCode]! as List<dynamic>)
                .map(
                  (e) => int.parse(
                    (e as Map<String, dynamic>)['error_data'] as String,
                  ),
                )
                .toSet();
          }
          return {};
        },
      );

  Future<Set<int>?> getLivingDevices() async => _getRequest(
        endpoint: 'living_devices',
        parser: (body) {
          final data = jsonDecode(body) as List<dynamic>;
          return data.map((e) => e as int).toSet();
        },
      );

  Future<List<String>?> getErrorLog() async => _postRequest(
        endpoint: 'logs/download',
        body: '"subscriber/robast_error_best_effort.log"',
        parser: (body) {
          return body.split('\n');
        },
      );

  Future<T?> _getRequest<T>({
    required String endpoint,
    required T? Function(String) parser,
  }) async {
    try {
      final header = {
        'Content-Type': 'application/json',
      };
      final response = await http.get(
        Uri.parse('$prefix/$endpoint'),
        headers: header,
      );
      if (response.statusCode == 200) {
        return parser(response.body);
      }
    } catch (e) {
      return null;
    }
    return null;
  }

  Future<T?> _postRequest<T>({
    required String endpoint,
    required T Function(String) parser,
    Object? body,
  }) async {
    try {
      final header = {
        'Content-Type': 'application/json',
      };
      final response = await http.post(
        Uri.parse('$prefix/$endpoint'),
        body: body,
        headers: header,
      );
      if (response.statusCode == 200) {
        return parser(response.body);
      }
    } catch (e) {
      return null;
    }
    return null;
  }
}
