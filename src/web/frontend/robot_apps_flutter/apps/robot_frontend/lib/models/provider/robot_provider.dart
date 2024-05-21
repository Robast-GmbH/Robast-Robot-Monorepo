import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotProvider extends ChangeNotifier {
  Pose? robotPose;
  List<DrawerModule>? modules;
  bool isNavigationBlocked = false;

  Timer? _robotPoseUpdateTimer;
  Timer? _modulesUpdateTimer;

  final _robotAPI = const RobotApiUtilities(prefix: 'http://192.168.0.200:8001');

  Future<void> updateIsNavigationBlocked() async {
    isNavigationBlocked = await _robotAPI.isNavigationBlocked();
    notifyListeners();
  }

  Future<void> updateRobotPose() async {
    robotPose = await _robotAPI.getRobotPose();
    notifyListeners();
  }

  Future<void> updateModules() async {
    modules = await _robotAPI.getModules();
    notifyListeners();
  }

  void startPeriodicIsNavigationBlockedUpdate(void Function({bool isBlocked}) onIsNavigationBlockedUpdate) {
    _robotPoseUpdateTimer = startPeriodicUpdate(
      () {
        updateIsNavigationBlocked();
        onIsNavigationBlockedUpdate(isBlocked: isNavigationBlocked);
      },
      const Duration(milliseconds: 100),
    );
  }

  void startPeriodicRobotPoseUpdate() {
    _robotPoseUpdateTimer = startPeriodicUpdate(
      updateRobotPose,
      const Duration(milliseconds: 500),
    );
  }

  void startPeriodicModulesUpdate() {
    _modulesUpdateTimer = startPeriodicUpdate(
      updateModules,
      const Duration(milliseconds: 500),
    );
  }

  void stopPeriodicModuleUpdate() {
    _modulesUpdateTimer?.cancel();
  }

  void stopPeriodicRobotPoseUpdate() {
    _robotPoseUpdateTimer?.cancel();
  }

  Timer startPeriodicUpdate(VoidCallback callback, Duration duration) {
    callback();
    return Timer.periodic(duration, (timer) {
      callback();
    });
  }

  void closeDrawer(DrawerModule module) {
    _robotAPI.closeDrawer(moduleID: module.moduleID, drawerID: module.drawerID);
  }

  void openDrawer(DrawerModule module) {
    _robotAPI.openDrawer(moduleID: module.moduleID, drawerID: module.drawerID);
  }

  Future<void> blockNavigation() async {
    await _robotAPI.blockNavigation();
  }

  Future<void> unblockNavigation() async {
    await _robotAPI.unblockNavigation();
  }
}
