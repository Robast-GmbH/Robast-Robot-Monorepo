import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class FleetProvider extends ChangeNotifier {
  FleetProvider({required String prefix}) {
    initMiddlewarAPI(prefix: prefix);
  }

  List<Robot> robots = [];
  Map<String, List<Submodule>> modules = {};
  Map<String, RobotTaskStatus?> tasks = {};
  bool isNavigationBlocked = false;
  Timer? _robotUpdateTimer;
  Timer? _moduleUpdateTimer;
  Timer? _isRobotNavigationBlockedUpdateTimer;

  final MiddlewareApiUtilities _middlewareApi = MiddlewareApiUtilities();

  void initMiddlewarAPI({required String prefix}) {
    _middlewareApi.setPrefix(prefix: prefix);
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
        robotModules.sort((a, b) => a.position.compareTo(b.position));
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
    _robotUpdateTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      debugPrint('Update Robots');
      await updateRobots();
    });
  }

  void startPeriodicModuleUpdate() {
    updateModules();
    _moduleUpdateTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      debugPrint('Update Modules');
      await updateModules();
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
}
