import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class FleetProvider extends ChangeNotifier {
  List<Robot> robots = [];
  Map<String, List<DrawerModule>> modules = {};
  Map<String, List<Task>> tasks = {};
  bool isNavigationBlocked = false;
  Timer? _robotUpdateTimer;
  Timer? _moduleUpdateTimer;
  Timer? _isRobotNavigationBlockedUpdateTimer;

  late MiddlewareApiUtilities _middlewareApi;

  Future<void> initMiddlewarAPI({required String prefix}) async {
    _middlewareApi = MiddlewareApiUtilities(prefix: prefix);
  }

  List<String> getIDsOfModules({required String robotName}) {
    return modules[robotName]?.map((e) => '${e.moduleID}_${e.drawerID}').toList() ?? [];
  }

  Future<void> updateProviderData() async {
    await updateRobots();
    await updateModules();
  }

  Future<void> updateRobots() async {
    robots = await _middlewareApi.getRobots();
    notifyListeners();
  }

  Future<void> updateModules() async {
    final updatedModules = <String, List<DrawerModule>>{};
    for (final robot in robots) {
      if (robot.name.isNotEmpty) {
        final robotModules = await _middlewareApi.getModules(robotName: robot.name);
        updatedModules[robot.name] = robotModules;
      }
    }
    modules = updatedModules;
    notifyListeners();
  }

  Future<void> updataTasks() async {
    final updatedTasks = <String, List<Task>>{};
    for (final robot in robots) {
      if (robot.name.isNotEmpty) {
        final robotTasks = await _middlewareApi.getTasks(robotName: robot.name);
        updatedTasks[robot.name] = robotTasks;
      }
    }
    tasks = updatedTasks;
    notifyListeners();
  }

  void startPeriodicRobotUpdate() {
    updateRobots();
    _robotUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      debugPrint('Update Robots');
      await updateRobots();
    });
  }

  void startPeriodicModuleUpdate() {
    updateModules();
    _moduleUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      debugPrint('Update Modules');
      await updateModules();
    });
  }

  void startPeriodicIsNavigationBlockedUpdate() {
    _isRobotNavigationBlockedUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      for (final robot in robots) {
        isNavigationBlocked = await _middlewareApi.isNavigationBlocked(robotName: robot.name);
      }
      notifyListeners();
    });
  }

  void stopPeriodicUpdates() {
    _robotUpdateTimer?.cancel();
    _moduleUpdateTimer?.cancel();
    _isRobotNavigationBlockedUpdateTimer?.cancel();
  }

  void stopPeriodicModuleUpdate() {
    _moduleUpdateTimer?.cancel();
  }

  Future<void> openDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    await _middlewareApi.openDrawer(
      robotName: robotName,
      moduleID: moduleID,
      drawerID: drawerID,
    );
    await updateModules();
  }

  Future<void> closeDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    await _middlewareApi.closeDrawer(
      robotName: robotName,
      moduleID: moduleID,
      drawerID: drawerID,
    );
    await updateModules();
  }

  Future<void> stopRobot({required String robotName}) async {
    await _middlewareApi.stopRobot(robotName: robotName);
  }

  Future<void> resumeRobot({required String robotName}) async {
    await _middlewareApi.resumeRobot(robotName: robotName);
  }
}
