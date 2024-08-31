import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class ChangePasswordPage extends StatelessWidget {
  ChangePasswordPage({super.key});

  final formKey = GlobalKey<FormState>();
  final oldPasswordController = TextEditingController();
  final newPasswordController = TextEditingController();
  final repeatNewPasswordController = TextEditingController();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Passwort ändern'),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () async {
          if (formKey.currentState!.validate()) {
            final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).changePassword(
              oldPassword: oldPasswordController.text,
              newPassword: newPasswordController.text,
            );
            if (wasSuccessful) {
              Navigator.pop(context);
            } else {
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                content: Text('Passwort konnte nicht geändert werden'),
              ));
            }
          }
        },
        child: const Icon(Icons.save),
      ),
      body: Form(
        key: formKey,
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            children: [
              CustomTextField(
                label: 'Altes Passwort',
                controller: oldPasswordController,
              ),
              SizedBox(height: 16),
              CustomTextField(
                label: 'Neues Passwort',
                controller: newPasswordController,
                validator: Validators.passwordValidator,
              ),
              SizedBox(height: 16),
              CustomTextField(
                label: 'Neues Passwort wiederholen',
                controller: repeatNewPasswordController,
                validator: (value) {
                  if (value != newPasswordController.text) {
                    return 'Passwörter stimmen nicht überein';
                  }
                  return null;
                },
              ),
            ],
          ),
        ),
      ),
    );
  }
}
