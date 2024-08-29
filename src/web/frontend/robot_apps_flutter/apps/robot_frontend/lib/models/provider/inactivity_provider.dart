import 'dart:async';
import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class InactivityProvider with ChangeNotifier {
  Timer? _inactivityTimer;
  final Duration _timeoutDuration = Duration(seconds: 600);
  final int _timeoutReactionDurationInS = 10;
  final GlobalKey<NavigatorState> _navigatorKey;

  InactivityProvider(this._navigatorKey);

  void cancelTimer() {
    _inactivityTimer?.cancel();
  }

  void resetInactivityTimer() {
    _inactivityTimer?.cancel();
    _inactivityTimer = Timer(_timeoutDuration, () async => _onInactivityTimeout());
  }

  Future<void> _onInactivityTimeout() async {
    if (_navigatorKey.currentContext == null) return;
    final context = _navigatorKey.currentContext!;

    bool isBarrierDismissed = true;
    int _counter = _timeoutReactionDurationInS;
    Timer? _timer;
    await showDialog(
      context: _navigatorKey.currentContext!,
      builder: (dialogContext) {
        return FractionallySizedBox(
          child: AlertDialog(
            backgroundColor: RobotColors.primaryBackground,
            title: const Text(
              'Inaktivitätszeit überschritten',
              style: TextStyle(
                color: RobotColors.primaryText,
                fontSize: 28,
              ),
            ),
            content: StatefulBuilder(builder: (context, setState) {
              if (_timer == null) {
                _timer = Timer.periodic(Duration(seconds: 1), (timer) {
                  _counter--;
                  if (_counter > 0) {
                    setState(() {});
                  } else {
                    timer.cancel();
                    isBarrierDismissed = false;
                    Navigator.popUntil(context, (route) => route.isFirst);
                  }
                });
              }
              return Text(
                'Sie waren zu lange inaktiv. Automatische Abmeldung in $_counter Sekunden.',
                style: TextStyle(
                  color: RobotColors.secondaryText,
                  fontSize: 24,
                ),
              );
            }),
            actions: [
              TextButton(
                onPressed: () {
                  isBarrierDismissed = false;
                  Navigator.popUntil(context, (route) => route.isFirst);
                },
                child: const Text(
                  'Abmelden',
                  style: TextStyle(
                    color: RobotColors.secondaryText,
                    fontSize: 24,
                  ),
                ),
              ),
              TextButton(
                onPressed: () {
                  isBarrierDismissed = false;
                  resetInactivityTimer();
                  Navigator.pop(dialogContext);
                },
                child: const Text(
                  'Abbrechen',
                  style: TextStyle(
                    color: RobotColors.secondaryText,
                    fontSize: 24,
                  ),
                ),
              ),
            ],
          ),
        );
      },
    ).then(
      (value) {
        _timer?.cancel();
        if (isBarrierDismissed) {
          resetInactivityTimer();
        }
      },
    );
  }

  @override
  void dispose() {
    _inactivityTimer?.cancel();
    super.dispose();
  }
}
