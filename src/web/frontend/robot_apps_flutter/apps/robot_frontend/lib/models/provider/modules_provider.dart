import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class ModulesProvider extends ChangeNotifier {
  List<RobotDrawer> _modules = [];
  List<RobotDrawer> get modules => _modules;
  Timer? _modulesUpdateTimer;

  late MiddlewareApiUtilities _middlewareApiUtilities;

  void initMiddlewareApiUtilities({required String prefix}) {
    _middlewareApiUtilities = MiddlewareApiUtilities(prefix: prefix);
  }

  Future<void> startModulesUpdateTimer() async {
    final modules = await _middlewareApiUtilities.modules.getModules(robotName: 'rb_theron');
    setModules(modules);
    _modulesUpdateTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      final modules = await _middlewareApiUtilities.modules.getModules(robotName: 'rb_theron');
      setModules(modules);
    });
  }

  void stopModulesUpdateTimer() {
    _modulesUpdateTimer?.cancel();
  }

  void setModules(List<RobotDrawer> modules) {
    _modules = modules;
    notifyListeners();
  }

  Future<bool> createModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required int position,
    required int size,
    required String variant,
  }) async {
    final wasSuccessful = await _middlewareApiUtilities.modules.createModule(
      robotName: robotName,
      moduleID: moduleID,
      drawerID: drawerID,
      position: position,
      size: size,
      variant: variant,
    );
    return wasSuccessful;
  }

  Future<bool> deleteModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final wasSuccessful = await _middlewareApiUtilities.modules.deleteModule(
      robotName: robotName,
      moduleID: moduleID,
      drawerID: drawerID,
    );
    return wasSuccessful;
  }
}
