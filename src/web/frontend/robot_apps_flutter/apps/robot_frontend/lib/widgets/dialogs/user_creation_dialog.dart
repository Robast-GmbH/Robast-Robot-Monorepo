import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class UserCreationDialog extends StatelessWidget {
  const UserCreationDialog({required this.firstNameController, required this.lastNameController, super.key});

  final TextEditingController firstNameController;
  final TextEditingController lastNameController;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: Colors.black.withOpacity(0.2),
      title: const Text(
        'Nutzererstellung',
        style: TextStyle(color: RobotColors.primaryText),
      ),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          TextField(
            controller: firstNameController,
            decoration: const InputDecoration(labelText: 'Vorname'),
            style: const TextStyle(color: RobotColors.secondaryText),
          ),
          TextField(
            controller: lastNameController,
            decoration: const InputDecoration(labelText: 'Nachname'),
            style: const TextStyle(color: RobotColors.secondaryText),
          ),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.pop(context, true);
          },
          child: const Text('Bestätigen'),
        ),
        TextButton(
          onPressed: () {
            Navigator.pop(context, false);
          },
          child: const Text('Abbrechen'),
        ),
      ],
    );
  }
}
