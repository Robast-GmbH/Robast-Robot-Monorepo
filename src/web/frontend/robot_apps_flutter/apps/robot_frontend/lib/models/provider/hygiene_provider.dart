import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class HygieneProvider extends ChangeNotifier {
  final _middlewareApiUtilities = MiddlewareApiUtilities();
  Timer? _hygieneDataUpdateTimer;
  bool? requiresCleaning;
  bool requiresDisinfectionAfterUsage = true;

  void startPeriodicHygieneDataUpdate() {
    _hygieneDataUpdateTimer?.cancel();
    _hygieneDataUpdateTimer = Timer.periodic(const Duration(seconds: 10), (timer) async {
      await updateRequiresCleaning(robotName: 'rb_theron');
      await updateRequiresDisinfectionAfterUsage(robotName: 'rb_theron');
    });
  }

  void stopPeriodicRequiresCleaningUpdate() {
    _hygieneDataUpdateTimer?.cancel();
  }

  Future<bool> setCycle({required String robotName, required double cycleTimeInH}) async {
    return await _middlewareApiUtilities.hygiene.setCycle(robotName: robotName, cycleTimeInH: cycleTimeInH);
  }

  Future<double?> getCycle({required String robotName}) async {
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

  Future<void> updateRequiresDisinfectionAfterUsage({required String robotName}) async {
    final result = await _middlewareApiUtilities.hygiene.getRequiresDisinfectionAfterUsage(robotName: robotName);
    if (result != null) {
      requiresDisinfectionAfterUsage = result;
      notifyListeners();
    }
  }
}
