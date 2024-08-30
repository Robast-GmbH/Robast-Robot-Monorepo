import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';

class BackgroundView extends StatelessWidget {
  const BackgroundView({
    required this.child,
    this.inactivityTimerEnabled = true,
    super.key,
  });
  final Widget child;
  final bool inactivityTimerEnabled;
  @override
  Widget build(BuildContext context) {
    if (inactivityTimerEnabled) {
      Provider.of<InactivityProvider>(context, listen: false).resetInactivityTimer();
    }
    return GestureDetector(
      onTapDown: inactivityTimerEnabled
          ? (details) {
              Provider.of<InactivityProvider>(context, listen: false).resetInactivityTimer();
            }
          : null,
      child: Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topCenter,
            end: Alignment.bottomCenter,
            colors: [RobotColors.secondaryBackground, RobotColors.primaryBackground],
          ),
        ),
        child: child,
      ),
    );
  }
}
