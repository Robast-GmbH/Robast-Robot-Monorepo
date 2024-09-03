import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/home_page.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class ChangePasswordPage extends StatelessWidget {
  ChangePasswordPage({this.forceChange = false, this.initialInput = '', super.key});

  final bool forceChange;
  final String initialInput;

  final formKey = GlobalKey<FormState>();
  final oldPasswordController = TextEditingController();
  final newPasswordController = TextEditingController();
  final repeatNewPasswordController = TextEditingController();
  @override
  Widget build(BuildContext context) {
    oldPasswordController.text = initialInput;
    return Scaffold(
      appBar: AppBar(
        title: Text(forceChange ? 'Ändern Sie Ihr Initialpasswort' : 'Passwort ändern'),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () async {
          if (formKey.currentState!.validate()) {
            final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).changePassword(
              oldPassword: oldPasswordController.text,
              newPassword: newPasswordController.text,
            );
            if (wasSuccessful) {
              if (forceChange) {
                Navigator.pushReplacement(context, MaterialPageRoute(builder: (context) => HomePage()));
              } else {
                Navigator.pop(context);
              }
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
              if (forceChange) ...[
                const Text(
                  'Um Ihre Sicherheit zu gewährleisten, müssen Sie Ihr Initialpasswort ändern, bevor Sie die App nutzen können. Dieser Schritt stellt sicher, dass Ihr Konto geschützt ist und nur Sie darauf zugreifen können.',
                  style: TextStyle(color: WebColors.secondaryText, fontSize: 18),
                ),
                SizedBox(height: 32),
              ],
              CustomTextField(
                label: 'Altes Passwort',
                controller: oldPasswordController,
                obsucureText: true,
              ),
              SizedBox(height: 16),
              CustomTextField(
                label: 'Neues Passwort',
                controller: newPasswordController,
                validator: Validators.passwordValidator,
                obsucureText: true,
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
                obsucureText: true,
              ),
              SizedBox(height: 16),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Row(
                    children: [
                      Icon(
                        Icons.info_outline,
                        color: WebColors.secondaryIcon,
                      ),
                      SizedBox(
                        width: 16,
                      ),
                      Expanded(
                        child: Text(
                          'Ihr Passwort muss mindestens 8 Zeichen lang sein, mindestens einen Großbuchstaben, einen Kleinbuchstaben, eine Ziffer und ein Sonderzeichen enthalten.',
                          style: TextStyle(color: WebColors.secondaryText, fontSize: 16),
                        ),
                      ),
                    ],
                  ),
                ),
              )
            ],
          ),
        ),
      ),
    );
  }
}
