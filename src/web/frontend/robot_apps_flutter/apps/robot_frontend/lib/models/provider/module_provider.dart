import 'dart:async';
import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class ModuleProvider extends ChangeNotifier {
  List<Submodule> _submodules = [];
  List<Submodule> get submodules => _submodules;
  Timer? _submodulesUpdateTimer;
  bool isInSubmoduleProcess = false;

  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<void> startSubmodulesUpdateTimer({VoidCallback? onModuleProcess}) async {
    _submodulesUpdateTimer?.cancel();
    final submodules = await _middlewareApiUtilities.modules.getSubmodules(robotName: 'rb_theron');
    _setSubmodules(submodules);

    _submodulesUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      await fetchSubmodules();
      if (_submodules.any((element) => element.moduleProcess.status != ModuleProcessStatus.idle)) {
        onModuleProcess?.call();
      }
    });
  }

  void stopSubmodulesUpdateTimer() {
    _submodulesUpdateTimer?.cancel();
  }

  Future<void> fetchSubmodules() async {
    final submodules = await _middlewareApiUtilities.modules.getSubmodules(robotName: 'rb_theron');
    _setSubmodules(submodules);
  }

  void _setSubmodules(List<Submodule> submodules) {
    _submodules = submodules;
    notifyListeners();
  }

  Future<bool> createSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
    required int position,
    required int size,
    required String variant,
  }) async {
    final wasSuccessful = await _middlewareApiUtilities.modules.createSubmodule(
      robotName: robotName,
      submoduleAddress: submoduleAddress,
      position: position,
      size: size,
      variant: variant,
    );
    return wasSuccessful;
  }

  Future<bool> deleteSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final wasSuccessful = await _middlewareApiUtilities.modules.deleteSubmodule(
      robotName: robotName,
      submoduleAddress: submoduleAddress,
    );
    return wasSuccessful;
  }

  Future<bool> startSubmoduleProcess({
    required SubmoduleAddress submoduleAddress,
    required String processName,
    required Map<String, int> itemsByChange,
  }) async {
    return _middlewareApiUtilities.modules.startSubmoduleProcess(
      robotName: 'rb_theron',
      submoduleAddress: submoduleAddress,
      processName: processName,
      itemsByChange: itemsByChange,
    );
  }

  Future<void> closeSubmodule(Submodule submodule) async {
    await _middlewareApiUtilities.modules.closeSubmodule(
      robotName: submodule.robotName,
      submoduleAddress: submodule.address,
    );
  }

  Future<void> openSubmodule(Submodule module) async {
    await _middlewareApiUtilities.modules.openSubmodule(
      robotName: module.robotName,
      submoduleAddress: module.address,
    );
  }

  Future<void> finishSubmoduleProcess(Submodule submoduleInProcess) async {
    await _middlewareApiUtilities.modules.finishSubmoduleProcess(
      robotName: submoduleInProcess.robotName,
      submoduleAddress: submoduleInProcess.address,
    );
  }

  Future<bool> reserveSubmodule({
    required SubmoduleAddress submoduleAddress,
    String taskID = '',
    List<String> userIDs = const [],
    List<String> userGroups = const [],
  }) async {
    return _middlewareApiUtilities.modules.reserveSubmodule(
      robotName: 'rb_theron',
      submoduleAddress: submoduleAddress,
      taskID: taskID,
      userIDs: userIDs,
      userGroups: userGroups,
    );
  }

  Future<bool> freeSubmodule({
    required SubmoduleAddress submoduleAddress,
  }) async {
    return _middlewareApiUtilities.modules.freeSubmodule(
      robotName: 'rb_theron',
      submoduleAddress: submoduleAddress,
    );
  }
}
