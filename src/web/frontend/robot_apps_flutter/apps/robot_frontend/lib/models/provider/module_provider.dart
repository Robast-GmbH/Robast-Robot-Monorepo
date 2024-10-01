import 'dart:async';
import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class ModuleProvider extends ChangeNotifier {
  List<List<Submodule>> _modules = [];
  List<List<Submodule>> get modules => _modules;

  List<Submodule> get submodules => _modules.fold([], (prev, module) {
        prev.addAll(module);
        return prev;
      });
  Timer? _modulesUpdateTimer;
  bool isInSubmoduleProcess = false;

  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<void> startSubmodulesUpdateTimer({VoidCallback? onModuleProcess}) async {
    _modulesUpdateTimer?.cancel();
    await fetchModules();

    _modulesUpdateTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) async {
      await fetchModules();
      if (submodules.any((element) => element.moduleProcess.status != ModuleProcessStatus.idle)) {
        onModuleProcess?.call();
      }
    });
  }

  void stopSubmodulesUpdateTimer() {
    _modulesUpdateTimer?.cancel();
  }

  Future<void> fetchModules() async {
    final submodules = await _middlewareApiUtilities.modules.getSubmodules(robotName: 'rb_theron');
    submodules.sort((a, b) => a.position.compareTo(b.position));
    _setModules(submodules);
  }

  void _setModules(List<Submodule> submodules) {
    final updatedModules = <List<Submodule>>[];
    int currentModuleIndex = 0;
    int positionIndex = -1;
    for (final submodule in submodules) {
      if (submodule.position != currentModuleIndex) {
        currentModuleIndex = submodule.position;
        updatedModules.add([submodule]);
        positionIndex++;
      } else {
        updatedModules[positionIndex].add(submodule);
      }
    }
    _modules = updatedModules;
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

  Future<void> cancelSubmoduleProcess(Submodule submoduleInProcess) async {
    await _middlewareApiUtilities.modules.cancelSubmoduleProcess(
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

  int getSubmodulePosition(Submodule submodule) {
    return _modules.indexWhere((element) => element.contains(submodule)) + 1;
  }
}
