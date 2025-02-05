import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';

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
                  "Not\u{2011}Aus\u{2011}Schalter gedrückt",
                  style: TextStyle(color: RobotColors.secondaryText, fontSize: 100),
                ),
                SizedBox(
                  height: 32,
                ),
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 64.0),
                  child: Row(
                    children: [
                      Icon(Icons.arrow_right_alt, color: Colors.transparent, size: 200),
                      Expanded(
                        child: Padding(
                          padding: EdgeInsets.symmetric(horizontal: 32),
                          child: Text(
                            'Zum Verschieben des Roboters den orangenen Knopf gedrückt halten.',
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              color: RobotColors.secondaryText,
                              fontSize: 64,
                            ),
                          ),
                        ),
                      ),
                      Icon(Icons.arrow_right_alt, color: RobotColors.secondaryText, size: 200),
                    ],
                  ),
                ),
                SizedBox(
                  height: 32,
                ),
                Padding(
                  padding: EdgeInsets.symmetric(horizontal: 64.0),
                  child: Row(
                    children: [
                      RotatedBox(quarterTurns: 2, child: Icon(Icons.arrow_right_alt, color: RobotColors.secondaryText, size: 200)),
                      Expanded(
                        child: Padding(
                          padding: EdgeInsets.symmetric(horizontal: 32),
                          child: Text(
                            'Zum Deaktivieren des Not\u{2011}Aus die Not\u{2011}Aus\u{2011}Schalter entriegeln und den blauen Knopf drücken.',
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              color: RobotColors.secondaryText,
                              fontSize: 64,
                            ),
                          ),
                        ),
                      ),
                      Icon(Icons.arrow_right_alt, color: Colors.transparent, size: 200),
                    ],
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
