import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:robot_api_utilities/robot_api_utilities.dart';

class NotificationProvider extends ChangeNotifier {
  final Set<int> _heartbeatTimeoutErrors = {};
  Set<int> get heartbeatTimeoutErrors => _heartbeatTimeoutErrors;
  Timer? _heartbeatTimeoutsUpdateTimer;

  late RobotApiUtilities _robotAPI;

  void initRobotAPI({required String prefix}) {
    _robotAPI = RobotApiUtilities(prefix: prefix);
  }

  void removeHearbeatTimeoutError(int error) {
    if (_heartbeatTimeoutErrors.contains(error)) {
      _heartbeatTimeoutErrors.remove(error);
      notifyListeners();
    }
  }

  void startPeriodicHeartbeatTimeoutsUpdate() {
    stopPeriodicHeartbeatTimeoutsUpdate();
    _heartbeatTimeoutsUpdateTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      final errors = await _robotAPI.getHeartbeatTimeoutErrors();
      final livingDevices = await _robotAPI.getLivingDevices();
      if (errors != null) {
        _heartbeatTimeoutErrors.addAll(errors);
        _heartbeatTimeoutErrors.removeAll(livingDevices ?? {});
        notifyListeners();
      }
    });
  }

  void stopPeriodicHeartbeatTimeoutsUpdate() {
    _heartbeatTimeoutsUpdateTimer?.cancel();
  }
}
