import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/dialogs/finish_cleaning_dialog.dart';

class CleaningPage extends StatefulWidget {
  const CleaningPage({super.key});

  @override
  State<CleaningPage> createState() => _CleaningPageState();
}

class _CleaningPageState extends State<CleaningPage> {
  final touchscreenID = 6;
  late final Future<bool> disableTouchscreenFuture;
  int disabledSeconds = 30;
  Timer? disabledTouchscreenTimer;

  void enableTouchscreen() {
    disabledTouchscreenTimer?.cancel();
    Process.run('xinput', [
      'enable',
      touchscreenID.toString(),
    ]);
  }

  Future<bool> disableTouchscreen() async {
    try {
      final result = await Process.run('xinput', [
        'disable',
        touchscreenID.toString(),
      ]);

      if (result.exitCode == 0) {
        disabledTouchscreenTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
          setState(() {
            disabledSeconds--;
          });
          if (disabledSeconds == 0) {
            enableTouchscreen();
            timer.cancel();
            await showDialog(
              barrierDismissible: false,
              context: context,
              builder: (context) => const FinishCleaningDialog(),
            );
          }
        });
        return true;
      } else {
        return false;
      }
    } catch (e) {
      return false;
    }
  }

  @override
  void initState() {
    super.initState();
    disableTouchscreenFuture = disableTouchscreen();
  }

  @override
  void dispose() {
    enableTouchscreen();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Reinigung',
      child: Center(
        child: FutureBuilder<bool>(
            future: disableTouchscreenFuture,
            builder: (context, snapshot) {
              if (snapshot.connectionState != ConnectionState.done) {
                return const CircularProgressIndicator();
              }
              final isTouchscreenDisabled = snapshot.data ?? false;
              if (isTouchscreenDisabled) {
                return Text(
                  'Touchscreen deaktiviert für $disabledSeconds Sekunden',
                  style: const TextStyle(fontSize: 80, color: RobotColors.primaryText),
                );
              } else {
                return const Text(
                  'Touchscreen konnte nicht deaktiviert werden',
                  style: TextStyle(fontSize: 80, color: RobotColors.primaryText),
                );
              }
            }),
      ),
    );
  }
}
