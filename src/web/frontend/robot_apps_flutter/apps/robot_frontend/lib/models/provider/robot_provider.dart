import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotProvider extends ChangeNotifier {
  Pose? _robotPose;
  bool _isRobotLost = false;
  bool get isRobotLost => _isRobotLost;
  BatteryStatus? _batteryStatus;
  double? get batteryLevel => _batteryStatus?.level;
  bool? get isCharging => _batteryStatus?.isCharging;

  int? _remainingDisinfections;
  int? get remainingDisinfections => _remainingDisinfections;

  bool? _isEmergencyStopPressed;
  bool? get isEmergencyStopPressed => _isEmergencyStopPressed;

  Timer? _batteryLevelUpdateTimer;
  Timer? _robotPoseUpdateTimer;
  Timer? _remainingDisinfectionsUpdateTimer;
  Timer? _isEmergencyStopPressedUpdateTimer;
  Timer? _isRobotLostUpdateTimer;

  late RobotApiUtilities _robotAPI;

  Pose? get robotPose => _robotPose;

  void initRobotAPI({required String prefix}) {
    _robotAPI = RobotApiUtilities(prefix: prefix);
  }

  Future<void> updateRobotPose() async {
    _robotPose = await _robotAPI.getRobotPose();
    notifyListeners();
  }

  Future<void> updateBatteryStatus() async {
    _batteryStatus = await _robotAPI.getBatteryStatus();
    notifyListeners();
  }

  Future<void> updateRemainingDisinfections() async {
    _remainingDisinfections = await _robotAPI.getRemainingDisinfections();
    notifyListeners();
  }

  Future<void> updateEmergencyStopPressed() async {
    _isEmergencyStopPressed = await _robotAPI.getEmergencyStopPressed();
    notifyListeners();
  }

  void startPeriodicIsEmergencyStopPressedUpdate() {
    stopPeriodicIsEmergencyStopPressedUpdate();
    startPeriodicUpdate(
      timerRef: _isEmergencyStopPressedUpdateTimer,
      callback: updateEmergencyStopPressed,
      duration: const Duration(milliseconds: 100),
    );
  }

  void startPeriodicRemainingDisinfectionsUpdate() {
    startPeriodicUpdate(
      timerRef: _remainingDisinfectionsUpdateTimer,
      callback: updateRemainingDisinfections,
      duration: const Duration(seconds: 30),
    );
  }

  void startPeriodicBatteryStatusUpdate() {
    startPeriodicUpdate(
      timerRef: _batteryLevelUpdateTimer,
      callback: updateBatteryStatus,
      duration: const Duration(minutes: 1),
    );
  }

  void startPeriodicRobotPoseUpdate() {
    startPeriodicUpdate(
      timerRef: _robotPoseUpdateTimer,
      callback: updateRobotPose,
      duration: const Duration(milliseconds: 500),
    );
  }

  void startPeriodicIsRobotLostUpdate() {
    startPeriodicUpdate(
      timerRef: _isRobotLostUpdateTimer,
      callback: updateIsRobotLost,
      duration: const Duration(seconds: 30),
    );
  }

  void stopPeriodicIsEmergencyStopPressedUpdate() {
    _isEmergencyStopPressedUpdateTimer?.cancel();
  }

  void stopPeriodicRemainingDisinfectionsUpdate() {
    _remainingDisinfectionsUpdateTimer?.cancel();
  }

  void stopPeriodicBatteryStatusUpdate() {
    _batteryLevelUpdateTimer?.cancel();
  }

  void stopPeriodicRobotPoseUpdate() {
    _robotPoseUpdateTimer?.cancel();
  }

  void stopPeriodicIsRobotLostUpdate() {
    _isRobotLostUpdateTimer?.cancel();
  }

  void startPeriodicUpdate({required Timer? timerRef, required VoidCallback callback, required Duration duration}) {
    timerRef?.cancel();
    callback();
    timerRef = Timer.periodic(duration, (timer) {
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
    final wasSuccessful = await _robotAPI.waitForDisinfectionTriggered(timeout: timeout) ?? false;
    if (wasSuccessful) {
      onDisinfection();
      updateRemainingDisinfections();
    }
    return wasSuccessful;
  }

  Future<bool> renewDisinfectionFluidContainer() async {
    return await _robotAPI.refillDisinfectionFluidContainer() ?? false;
  }

  Future<void> updateIsRobotLost() async {
    _isRobotLost = await _robotAPI.getIsRobotLost() ?? false;
    notifyListeners();
  }

  Future<List<String>?> getErrorProtocol() async {
    final errorLog = await _robotAPI.getErrorLog();
    return errorLog?.where((logMessage) => logMessage.contains('error_code')).toList().reversed.toList();
  }
}
