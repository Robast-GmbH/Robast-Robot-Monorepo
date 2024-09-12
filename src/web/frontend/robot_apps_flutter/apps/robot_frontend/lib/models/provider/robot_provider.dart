import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotProvider extends ChangeNotifier {
  Pose? _robotPose;
  bool _isRobotLost = false;
  bool get isRobotLost => _isRobotLost;

  Timer? _robotPoseUpdateTimer;

  late RobotApiUtilities _robotAPI;

  Pose? get robotPose => _robotPose;

  void initRobotAPI({required String prefix}) {
    _robotAPI = RobotApiUtilities(prefix: prefix);
  }

  Future<void> updateRobotPose() async {
    _robotPose = await _robotAPI.getRobotPose();
    notifyListeners();
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
    }
    return wasSuccessful;
  }

  Future<void> updateIsRobotLost() async {
    _isRobotLost = await _robotAPI.getIsRobotLost();
    notifyListeners();
  }
}
