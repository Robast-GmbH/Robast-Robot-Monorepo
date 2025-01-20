import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class FireAlarmPage extends StatelessWidget {
  const FireAlarmPage({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      collapsedTitle: true,
      showBackButton: false,
      inactivityTimerEnabled: false,
      ignoreMissingEmergencyStopData: true,
      child: ColoredBox(
        color: RobotColors.error,
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Text(
                    'Feueralarm',
                    style: TextStyle(
                      color: RobotColors.secondaryText,
                      fontSize: 100,
                    ),
                  ),
                  SizedBox(
                    width: 32,
                  ),
                  Icon(
                    Icons.fireplace_sharp,
                    size: 120,
                  )
                ],
              ),
              SizedBox(
                height: 32,
              ),
              Padding(
                padding: EdgeInsets.symmetric(horizontal: 128.0),
                child: Text(
                  'Zum Anhalten und Verschieben des Roboters Not\u{2011}Aus\u{2011}Schalter betätigen.',
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: RobotColors.secondaryText,
                    fontSize: 64,
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
