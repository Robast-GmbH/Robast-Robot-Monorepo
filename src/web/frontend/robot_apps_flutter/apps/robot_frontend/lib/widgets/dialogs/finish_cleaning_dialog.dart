// ignore_for_file: use_build_context_synchronously

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/hygiene_provider.dart';

class FinishCleaningDialog extends StatelessWidget {
  const FinishCleaningDialog({super.key});

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: RobotColors.primaryBackground,
      title: const Text(
        'Reinigung abschließen',
        style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
      ),
      content: const Text(
        'Hiermit bestätige ich, dass der Roboter sachgemäß gereinigt wurde.',
        style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
      ),
      actions: [
        TextButton(
          onPressed: () async {
            final hygieneProvider = Provider.of<HygieneProvider>(context, listen: false);
            await hygieneProvider.setLastCleaning(robotName: "rb_theron");
            await hygieneProvider.getLastCleaning(robotName: "rb_theron");
            await hygieneProvider.updateHygieneData(robotName: "rb_theron");
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
            Navigator.pop(context);
          },
          child: const Text(
            'Abbrechen',
            style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
          ),
        ),
      ],
    );
  }
}
