import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotProvider extends ChangeNotifier {
  Pose? _robotPose;
  bool _isRobotLost = false;
  bool get isRobotLost => _isRobotLost;
  double? _batteryLevel;
  double? get batteryLevel => _batteryLevel;

  int? _remainingDisinfections;
  int? get remainingDisinfections => _remainingDisinfections;

  Timer? _batteryLevelUpdateTimer;
  Timer? _robotPoseUpdateTimer;
  Timer? _remainingDisinfectionsUpdateTimer;

  late RobotApiUtilities _robotAPI;

  Pose? get robotPose => _robotPose;

  void initRobotAPI({required String prefix}) {
    _robotAPI = RobotApiUtilities(prefix: prefix);
  }

  Future<void> updateRobotPose() async {
    _robotPose = await _robotAPI.getRobotPose();
    notifyListeners();
  }

  Future<void> updateBatteryLevel() async {
    _batteryLevel = await _robotAPI.getBatteryLevel();
    notifyListeners();
  }

  Future<void> updateRemainingDisinfections() async {
    _remainingDisinfections = await _robotAPI.getRemainingDisinfections();
    notifyListeners();
  }

  void startPeriodicRemainingDisinfectionsUpdate() {
    _remainingDisinfectionsUpdateTimer = startPeriodicUpdate(
      updateRemainingDisinfections,
      const Duration(seconds: 30),
    );
  }

  void startPeriodicBatteryLevelUpdate() {
    _batteryLevelUpdateTimer = startPeriodicUpdate(
      updateBatteryLevel,
      const Duration(minutes: 1),
    );
  }

  void startPeriodicRobotPoseUpdate() {
    _robotPoseUpdateTimer = startPeriodicUpdate(
      updateRobotPose,
      const Duration(milliseconds: 500),
    );
  }

  void startPeriodicIsRobotLostUpdate() {
    _robotPoseUpdateTimer = startPeriodicUpdate(
      updateIsRobotLost,
      const Duration(seconds: 5),
    );
  }

  void stopPeriodicRemainingDisinfectionsUpdate() {
    _remainingDisinfectionsUpdateTimer?.cancel();
  }

  void stopPeriodicBatteryLevelUpdate() {
    _batteryLevelUpdateTimer?.cancel();
  }

  void stopPeriodicRobotPoseUpdate() {
    _robotPoseUpdateTimer?.cancel();
  }

  void stopPeriodicIsRobotLostUpdate() {
    _robotPoseUpdateTimer?.cancel();
  }

  Timer startPeriodicUpdate(VoidCallback callback, Duration duration) {
    callback();
    return Timer.periodic(duration, (timer) {
      callback();
    });
  }

  Future<void> blockNavigation() async {
    await _robotAPI.blockNavigation();
  }

  Future<void> unblockNavigation() async {
    await _robotAPI.unblockNavigation();
  }

  Future<bool> waitForDisinfectionTriggered({required VoidCallback onDisinfection, int timeout = 10}) async {
    final wasSuccessful = await _robotAPI.waitForDisinfectionTriggered(timeout: timeout);
    if (wasSuccessful) {
      onDisinfection();
      updateRemainingDisinfections();
    }
    return wasSuccessful;
  }

  Future<bool> renewDisinfectionFluidContainer() async {
    return await _robotAPI.refillDisinfectionFluidContainer();
  }

  Future<void> updateIsRobotLost() async {
    _isRobotLost = await _robotAPI.getIsRobotLost();
    notifyListeners();
  }
}
