import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotProvider extends ChangeNotifier {
  Pose? _robotPose;
  List<DrawerModule>? _modules;
  ModuleProcess? _moduleProcess;
  bool _isNavigationBlocked = false;

  Timer? _robotPoseUpdateTimer;
  Timer? _modulesUpdateTimer;

  late RobotApiUtilities _robotAPI;

  Pose? get robotPose => _robotPose;
  List<DrawerModule>? get modules => _modules;
  ModuleProcess? get moduleProcess => _moduleProcess;

  Future<void> initRobotAPI({required String prefix}) async {
    _robotAPI = RobotApiUtilities(prefix: prefix);
  }

  Future<void> updateIsNavigationBlocked() async {
    _isNavigationBlocked = await _robotAPI.isNavigationBlocked();
    notifyListeners();
  }

  Future<void> updateRobotPose() async {
    _robotPose = await _robotAPI.getRobotPose();
    notifyListeners();
  }

  Future<void> updateModules() async {
    _modules = await _robotAPI.getModules();
    _moduleProcess = await _robotAPI.getModuleProcess();
    notifyListeners();
  }

  void startPeriodicIsNavigationBlockedUpdate(void Function({bool isBlocked}) onIsNavigationBlockedUpdate) {
    _robotPoseUpdateTimer = startPeriodicUpdate(
      () {
        updateIsNavigationBlocked();
        onIsNavigationBlockedUpdate(isBlocked: _isNavigationBlocked);
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

  Future<void> finishModuleProcess() async {
    await _robotAPI.finishModuleProcess();
  }
}
