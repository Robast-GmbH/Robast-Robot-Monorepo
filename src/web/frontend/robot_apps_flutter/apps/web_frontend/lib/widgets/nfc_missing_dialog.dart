import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';

class NfcMissingDialog extends StatelessWidget {
  const NfcMissingDialog({required this.identifiers, super.key});

  final List<String> identifiers;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(
        'Nutzer ${identifiers.length != 1 ? 'haben' : 'hat'} keine NFC-ID',
        style: const TextStyle(color: WebColors.primaryText, fontSize: 24),
      ),
      content: Text(
        identifiers.length != 1
            ? '${identifiers.sublist(0, identifiers.length - 1).join(', ')} und ${identifiers.last} haben keine NFC-ID. Bitte wenden Sie sich für die Zuweisung einer NFC-ID an einen Admin.'
            : 'Der ${identifiers.first} hat keine NFC-ID. Bitte wenden Sie sich für die Zuweisung einer NFC-ID an einen Admin.',
        style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: const Text('OK', style: TextStyle(color: WebColors.secondaryText, fontSize: 18)),
        ),
      ],
    );
  }
}
