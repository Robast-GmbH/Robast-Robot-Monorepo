import 'dart:async';

import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/data/actions/robot_action.dart';
import 'package:web_interface_flutter/models/data/actions/waypoint.dart';
import 'package:web_interface_flutter/models/data/drawer_module.dart';
import 'package:web_interface_flutter/models/data/robot.dart';
import 'package:web_interface_flutter/models/data/task.dart';
import 'package:web_interface_flutter/models/data/user.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class RobotProvider extends ChangeNotifier {
  final List<User> users = [];
  final List<Task> tasks = [];
  List<Robot> robots = [];
  Map<String, List<DrawerModule>> modules = {};
  bool isAdmin = true;
  bool isRobotMoving = false;
  Task? currentTask;
  Timer? robotUpdateTimer;
  Timer? moduleUpdateTimer;

  Future<void> updateProviderData() async {
    await updateUsers();
    await updateRobots();
    await updateModules();
    await updateTasks();
  }

  Future<void> updateUsers() async {
    users.clear();
    users.addAll(await APIService.getUsers());
  }

  Future<void> updateRobots() async {
    robots = await APIService.getRobots();
    if (robots.isNotEmpty && robots.first.taskID != null) {
      final currentTask = await APIService.getTaskByID(robots.first.taskID!);
      isRobotMoving = currentTask?.actions.firstWhere(
        (element) => !element.isFinished,
        orElse: () => RobotAction(isFinished: false, step: 0),
      ) is Waypoint;
    } else {
      currentTask = null;
      isRobotMoving = false;
    }
    notifyListeners();
  }

  Future<void> updateTasks() async {
    tasks.clear();
    tasks.addAll(await APIService.getTasks());
  }

  Future<void> updateModules() async {
    final updatedModules = <String, List<DrawerModule>>{};
    for (final robot in robots) {
      if (robot.name.isNotEmpty) {
        final robotModules = await APIService.getModules(robotName: robot.name);
        updatedModules[robot.name] = robotModules;
      }
    }
    modules = updatedModules;
    notifyListeners();
  }

  void startPeriodicRobotUpdate() {
    updateRobots();
    robotUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      debugPrint("Update Robots");
      updateRobots();
    });
  }

  void startPeriodicModuleUpdate() {
    updateModules();
    moduleUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      debugPrint("Update Modules");
      updateModules();
    });
  }

  void stopPeriodicUpdates() {
    robotUpdateTimer?.cancel();
    moduleUpdateTimer?.cancel();
  }

  void stopPeriodicModuleUpdate() {
    moduleUpdateTimer?.cancel();
  }

  String generateTaskID() {
    return tasks.length.toString();
  }
}
