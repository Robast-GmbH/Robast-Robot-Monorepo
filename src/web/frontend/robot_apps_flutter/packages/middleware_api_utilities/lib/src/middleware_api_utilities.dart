import 'dart:convert';
import 'dart:typed_data';

import 'package:middleware_api_utilities/src/services/request_service.dart';
import 'package:middleware_api_utilities/src/sub_apis/fire_alarm_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/hygiene_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/logs_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/manuals_api.dart';
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
    _manuals = ManualsApi(prefix: prefix);
    _hygiene = HygieneApi(prefix: prefix);
    _fireAlarm = FireAlarmApi(prefix: prefix);
    _logs = LogsApi(prefix: prefix);
  }

  late String _prefix;
  late ModulesApi _modules;
  late UsersApi _users;
  late TasksApi _tasks;
  late NFCApi _nfc;
  late ManualsApi _manuals;
  late HygieneApi _hygiene;
  late FireAlarmApi _fireAlarm;
  late LogsApi _logs;

  String get prefix => _prefix;
  ModulesApi get modules => _modules;
  UsersApi get users => _users;
  TasksApi get tasks => _tasks;
  NFCApi get nfc => _nfc;
  ManualsApi get manuals => _manuals;
  HygieneApi get hygiene => _hygiene;
  FireAlarmApi get fireAlarm => _fireAlarm;
  LogsApi get logs => _logs;

  Future<bool> testConnection({
    required String url,
  }) async {
    try {
      final response = await RequestService.tryGet(uri: Uri.parse('$url/'));
      return response != null && response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }

  Future<List<Robot>> getRobots() async {
    final robots = <Robot>[];
    try {
      final response = await RequestService.tryGet(uri: Uri.parse('$_prefix/fleet'));
      if (response != null) {
        final jsonData = (jsonDecode(response.body) as Map<String, dynamic>)['fleet'] as List<dynamic>;
        for (final robot in jsonData) {
          final posReponse = await RequestService.tryGet(uri: Uri.parse('$_prefix/robot_pos?robot_name=$robot'));
          if (posReponse == null) {
            continue;
          }
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

  Future<BuildingMap?> getMap() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$_prefix/building_map'), timeoutInMS: 2000);
    if (response != null) {
      final data = RequestService.responseToMap(response: response);
      return BuildingMap.fromJson(data);
    } else {
      return null;
    }
  }

  Future<Uint8List?> getMapImage() async {
    final response = await RequestService.tryGet(uri: Uri.parse('$_prefix/building_map.png'), timeoutInMS: 3000);
    if (response != null) {
      final data = response.bodyBytes;
      return data;
    } else {
      return null;
    }
  }
}
