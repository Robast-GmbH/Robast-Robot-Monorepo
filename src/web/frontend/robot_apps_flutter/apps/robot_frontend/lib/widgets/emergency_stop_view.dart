import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class EmergencyStopView extends StatelessWidget {
  const EmergencyStopView({super.key});

  @override
  Widget build(BuildContext context) {
    return const ColoredBox(
      color: RobotColors.error,
      child: Center(
          child: Text(
        "Not-Aus Schalter gedrückt",
        style: TextStyle(color: RobotColors.secondaryText, fontSize: 80),
      )),
    );
  }
}
