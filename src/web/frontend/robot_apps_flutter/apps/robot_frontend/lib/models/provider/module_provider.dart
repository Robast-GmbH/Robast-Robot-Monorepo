import 'dart:async';
import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class ModuleProvider extends ChangeNotifier {
  List<RobotDrawer> _modules = [];
  List<RobotDrawer> get modules => _modules;
  Timer? _modulesUpdateTimer;
  bool isInModuleProcess = false;

  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<void> startModulesUpdateTimer({VoidCallback? onModuleProcess}) async {
    _modulesUpdateTimer?.cancel();
    final modules = await _middlewareApiUtilities.modules.getModules(robotName: 'rb_theron');
    setModules(modules);

    _modulesUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      await fetchModules();
      if (_modules.any((element) => element.moduleProcess.status != ModuleProcessStatus.idle)) {
        onModuleProcess?.call();
      }
    });
  }

  void stopModulesUpdateTimer() {
    _modulesUpdateTimer?.cancel();
  }

  void setModules(List<RobotDrawer> modules) {
    _modules = modules;
    notifyListeners();
  }

  Future<void> fetchModules() async {
    final modules = await _middlewareApiUtilities.modules.getModules(robotName: 'rb_theron');
    setModules(modules);
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

  Future<bool> startModuleProcess({
    required DrawerAddress drawerAddress,
    required String processName,
    required Map<String, int> itemsByChange,
  }) async {
    return _middlewareApiUtilities.modules.startModuleProcess(
      robotName: 'rb_theron',
      drawerAddress: drawerAddress,
      processName: processName,
      itemsByChange: itemsByChange,
    );
  }

  Future<void> closeDrawer(RobotDrawer module) async {
    await _middlewareApiUtilities.modules.closeDrawer(
      robotName: module.robotName,
      moduleID: module.moduleID,
      drawerID: module.drawerID,
    );
  }

  Future<void> openDrawer(RobotDrawer module) async {
    await _middlewareApiUtilities.modules.openDrawer(
      robotName: module.robotName,
      moduleID: module.moduleID,
      drawerID: module.drawerID,
    );
  }

  Future<void> finishModuleProcess(RobotDrawer moduleInProcess) async {
    await _middlewareApiUtilities.modules.finishModuleProcess(
      robotName: moduleInProcess.robotName,
      moduleID: moduleInProcess.moduleID,
      drawerID: moduleInProcess.drawerID,
    );
  }
}
