import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';

class FinishModuleSetupDialog extends StatelessWidget {
  const FinishModuleSetupDialog({super.key});

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: RobotColors.primaryBackground,
      title: const Text(
        'Modul Setup abschließen',
        style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
      ),
      content: const Text(
        'Hiermit bestätige ich, dass alle Steckplätze sachgemäß belegt sind.',
        style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.pop(context);
            Navigator.pop(context);
          },
          child: const Text(
            'Bestätigen',
            style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
          ),
        ),
        TextButton(
          onPressed: () {
            Navigator.pop(context);
          },
          child: const Text(
            'Zurück',
            style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
          ),
        ),
      ],
    );
  }
}
