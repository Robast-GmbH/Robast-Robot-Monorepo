import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class BackgroundView extends StatelessWidget {
  const BackgroundView({required this.child, super.key});
  final Widget child;
  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [RobotColors.secondaryBackground, RobotColors.primaryBackground],
        ),
      ),
      child: child,
    );
  }
}
