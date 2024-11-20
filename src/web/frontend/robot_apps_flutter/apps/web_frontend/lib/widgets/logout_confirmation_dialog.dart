import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/login_page.dart';

class LogoutConfirmationDialog extends StatelessWidget {
  const LogoutConfirmationDialog({super.key});

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text(
        'Wirklich ausloggen?',
        style: TextStyle(color: Colors.white),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
            Provider.of<UserProvider>(context, listen: false).user = null;
            Navigator.pushReplacement(context, MaterialPageRoute<LoginPage>(builder: (context) => LoginPage()));
          },
          child: const Text('Bestätigen'),
        ),
        TextButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: const Text('Abbrechen'),
        ),
      ],
    );
  }
}
