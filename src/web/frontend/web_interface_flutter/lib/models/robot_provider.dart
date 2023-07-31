import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot.dart';
import 'package:web_interface_flutter/models/task.dart';
import 'package:web_interface_flutter/models/user.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class RobotProvider extends ChangeNotifier {
  final List<User> users = [];
  final List<Robot> robots = [];
  final Map<String, Task> tasks = {};
  final Map<String, List<DrawerModule>> modules = {};

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
    robots.clear();
    robots.addAll(await APIService.getRobots());
  }

  Future<void> updateTasks() async {
    tasks.clear();
    final updatedTasks = await APIService.getTasks();
    updatedTasks.map((e) => tasks[e.robotName] = e);
  }

  Future<void> updateModules() async {
    modules.clear();
    for (final robot in robots) {
      final robotModules = await APIService.getModules(robot.name);
      modules[robot.name] = robotModules;
    }
  }
}
