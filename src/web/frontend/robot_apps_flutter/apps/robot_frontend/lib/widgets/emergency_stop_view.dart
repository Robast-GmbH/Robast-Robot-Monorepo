import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class EmergencyStopView extends StatelessWidget {
  const EmergencyStopView({super.key});

  @override
  Widget build(BuildContext context) {
    return const ColoredBox(
      color: RobotColors.error,
      child: Stack(
        fit: StackFit.expand,
        children: [
          Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  "Not-Aus Schalter gedrückt",
                  style: TextStyle(color: RobotColors.secondaryText, fontSize: 120),
                ),
                SizedBox(
                  height: 32,
                ),
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 128.0),
                  child: Text(
                    'Zum Verschieben des Roboters den orangenen Knopf gedrückt halten.',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: RobotColors.secondaryText,
                      fontSize: 80,
                    ),
                  ),
                ),
                SizedBox(
                  height: 16,
                ),
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 128.0),
                  child: Text(
                    'Zum Deaktivieren des Not-Aus die Not-Ause entriegeln und den blauen Knopf drücken.',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: RobotColors.secondaryText,
                      fontSize: 80,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
