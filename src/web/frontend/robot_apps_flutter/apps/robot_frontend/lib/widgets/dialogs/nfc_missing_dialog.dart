import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';

class NfcMissingDialog extends StatelessWidget {
  const NfcMissingDialog({required this.identifiers, super.key});

  final List<String> identifiers;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: RobotColors.secondaryBackground,
      title: Text(
        'Nutzer ${identifiers.length != 1 ? 'haben' : 'hat'} keine NFC-ID',
        style: const TextStyle(color: RobotColors.primaryText, fontSize: 28),
      ),
      content: Text(
        identifiers.length != 1
            ? 'Die Registrierung von ${identifiers.sublist(0, identifiers.length - 1).join(', ')} und ${identifiers.last} wurde noch nicht vollständig abgeschlossen (Authentifizierung). Bitte wenden Sie sich für die Einrichtung an einen Admin.'
            : 'Die Registrierung von ${identifiers.first} wurde noch nicht vollständig abgeschlossen (Authentifizierung). Bitte wenden Sie sich für die Einrichtung an einen Admin.',
        style: const TextStyle(color: RobotColors.secondaryText, fontSize: 24),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: const Text('OK', style: TextStyle(color: RobotColors.secondaryText, fontSize: 24)),
        ),
      ],
    );
  }
}
