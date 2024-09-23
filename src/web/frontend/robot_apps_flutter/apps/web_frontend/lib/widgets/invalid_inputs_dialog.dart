import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';

class InvalidInputsDialog extends StatelessWidget {
  const InvalidInputsDialog({super.key});

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text(
        'Ungültige Eingaben',
        style: TextStyle(color: WebColors.primaryText),
      ),
      content: const Text(
        'Bitte überprüfen Sie ihre Eingaben.',
        style: TextStyle(color: WebColors.primaryText),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: const Text('OK'),
        ),
      ],
    );
  }
}
