import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/fire_alarm_page.dart';

class FireAlarmProvider extends ChangeNotifier {
  FireAlarmProvider(this._navigatorKey);
  final _updateInterval = const Duration(seconds: 1);
  final GlobalKey<NavigatorState> _navigatorKey;
  final _middlewareApiUtilities = MiddlewareApiUtilities();
  Timer? _fireAlarmUpdateTimer;
  bool _fireAlarmReactionTriggered = false;

  void startPeriodicFireAlarmUpdate() {
    stopPeriodicFireAlarmUpdate();
    _fireAlarmUpdateTimer = Timer.periodic(_updateInterval, (timer) async {
      final fireAlarmTriggered = await _middlewareApiUtilities.fireAlarm.fireAlarmTriggered();
      if (fireAlarmTriggered == null) return;
      if (fireAlarmTriggered && !_fireAlarmReactionTriggered) {
        _fireAlarmReactionTriggered = true;
        final currentContext = _navigatorKey.currentContext!;
        final moduleProvider = Provider.of<ModuleProvider>(currentContext, listen: false);
        final userProvider = Provider.of<UserProvider>(currentContext, listen: false);
        final robotProvider = Provider.of<RobotProvider>(currentContext, listen: false);
        moduleProvider.cancelAllActiveModuleProcesses();
        userProvider.endUserSession(robotName: 'rb_theron');
        robotProvider.unblockNavigation();
        _navigatorKey.currentState!.popUntil((route) => route.isFirst);
        _navigatorKey.currentState!.push(MaterialPageRoute(builder: (context) => const FireAlarmPage()));
      } else if (!fireAlarmTriggered && _fireAlarmReactionTriggered) {
        _fireAlarmReactionTriggered = false;
        _navigatorKey.currentState!.pop();
      }
    });
  }

  void stopPeriodicFireAlarmUpdate() {
    _fireAlarmUpdateTimer?.cancel();
  }
}
