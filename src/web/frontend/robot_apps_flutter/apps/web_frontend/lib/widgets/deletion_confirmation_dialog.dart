import 'package:flutter/material.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class DeletionConfirmationDialog extends StatelessWidget {
  DeletionConfirmationDialog({required this.requiredTextInput, super.key});
  final formKey = GlobalKey<FormState>();
  final String requiredTextInput;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text(
        'Nutzer wirklich löschen?',
        style: TextStyle(color: Colors.white),
      ),
      content: Form(
        key: formKey,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Zum Bestätigen "$requiredTextInput" eingeben:',
              style: const TextStyle(color: Colors.white, fontSize: 18),
            ),
            const SizedBox(
              height: 8,
            ),
            CustomTextField(
              label: 'Erforderliche Eingabe',
              validator: (value) {
                if (value != requiredTextInput) {
                  return 'Texte stimmen nicht überein';
                }
                return null;
              },
            ),
          ],
        ),
      ),
      actions: [
        TextButton(
          child: const Text('Bestätigen'),
          onPressed: () {
            if (formKey.currentState!.validate()) {
              Navigator.of(context).pop(true);
            }
          },
        ),
        TextButton(
          child: const Text('Abbrechen'),
          onPressed: () {
            Navigator.of(context).pop(false);
          },
        ),
      ],
    );
  }
}
