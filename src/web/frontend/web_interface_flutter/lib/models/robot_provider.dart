import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot.dart';
import 'package:web_interface_flutter/models/task.dart';
import 'package:web_interface_flutter/models/user.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class RobotProvider extends ChangeNotifier {
  final List<User> users = [];
  List<Robot> robots = [];
  final Map<String, Task> tasks = {};
  Map<String, List<DrawerModule>> modules = {};

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
    notifyListeners();
  }

  Future<void> updateTasks() async {
    tasks.clear();
    final updatedTasks = await APIService.getTasks();
    updatedTasks.map((e) => tasks[e.robotName] = e);
  }

  Future<void> updateModules() async {
    final updatedModules = <String, List<DrawerModule>>{};
    for (final robot in robots) {
      final robotModules = await APIService.getModules(robot.name);
      updatedModules[robot.name] = robotModules;
    }
    modules = updatedModules;
    notifyListeners();
  }
}
