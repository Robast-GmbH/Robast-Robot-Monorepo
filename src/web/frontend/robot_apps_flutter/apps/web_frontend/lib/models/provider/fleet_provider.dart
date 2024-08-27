import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class FleetProvider extends ChangeNotifier {
  List<Robot> robots = [];
  Map<String, List<Submodule>> modules = {};
  Map<String, RobotTaskStatus?> tasks = {};
  bool isNavigationBlocked = false;
  Timer? _robotUpdateTimer;
  Timer? _moduleUpdateTimer;
  Timer? _isRobotNavigationBlockedUpdateTimer;

  late MiddlewareApiUtilities _middlewareApi;

  Future<void> initMiddlewarAPI({required String prefix}) async {
    _middlewareApi = MiddlewareApiUtilities();
  }

  List<String> getIDsOfModules({required String robotName}) {
    return modules[robotName]?.map((e) => '${e.address.moduleID}_${e.address.submoduleID}').toList() ?? [];
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
    final updatedModules = <String, List<Submodule>>{};
    for (final robot in robots) {
      if (robot.name.isNotEmpty) {
        final robotModules = await _middlewareApi.modules.getSubmodules(robotName: robot.name);
        updatedModules[robot.name] = robotModules;
      }
    }
    modules = updatedModules;
    notifyListeners();
  }

  Future<void> updataTasks() async {
    final updatedTasks = <String, RobotTaskStatus?>{};
    for (final robot in robots) {
      if (robot.name.isNotEmpty) {
        final robotTasks = await _middlewareApi.tasks.getRobotTasks(robotName: robot.name);
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

  Future<void> openSubmodule({
    required String robotName,
    required int moduleID,
    required int submoduleID,
  }) async {
    await _middlewareApi.modules.openSubmodule(
      robotName: robotName,
      submoduleAddress: SubmoduleAddress(moduleID: moduleID, submoduleID: submoduleID),
    );
    await updateModules();
  }

  Future<void> closeSubmodule({
    required String robotName,
    required int moduleID,
    required int submoduleID,
  }) async {
    await _middlewareApi.modules.closeSubmodule(
      robotName: robotName,
      submoduleAddress: SubmoduleAddress(moduleID: moduleID, submoduleID: submoduleID),
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
