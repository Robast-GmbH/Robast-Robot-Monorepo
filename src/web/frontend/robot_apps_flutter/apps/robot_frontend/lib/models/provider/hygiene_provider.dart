import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class HygieneProvider extends ChangeNotifier {
  final _middlewareApiUtilities = MiddlewareApiUtilities();
  Timer? _requiresCleaningUpdateTimer;
  bool? requiresCleaning;

  void startPeriodicRequiresCleaningUpdate() {
    _requiresCleaningUpdateTimer?.cancel();
    _requiresCleaningUpdateTimer = Timer.periodic(const Duration(seconds: 10), (timer) async {
      await updateRequiresCleaning(robotName: 'rb_theron');
    });
  }

  void stopPeriodicRequiresCleaningUpdate() {
    _requiresCleaningUpdateTimer?.cancel();
  }

  Future<bool> setCycle({required String robotName, required int cycleTimeInH}) async {
    return await _middlewareApiUtilities.hygiene.setCycle(robotName: robotName, cycleTimeInH: cycleTimeInH);
  }

  Future<int?> getCycle({required String robotName}) async {
    return await _middlewareApiUtilities.hygiene.getCycle(robotName: robotName);
  }

  Future<bool> setLastCleaning({required String robotName}) async {
    return await _middlewareApiUtilities.hygiene.setLastCleaning(robotName: robotName);
  }

  Future<DateTime?> getLastCleaning({required String robotName}) async {
    return await _middlewareApiUtilities.hygiene.getLastCleaning(robotName: robotName);
  }

  Future<void> updateRequiresCleaning({required String robotName}) async {
    requiresCleaning = await _middlewareApiUtilities.hygiene.getRequiresCleaning(robotName: robotName);
    notifyListeners();
  }
}
