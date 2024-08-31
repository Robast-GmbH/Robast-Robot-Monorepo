import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/change_password_page.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class UserSettingsPage extends StatelessWidget {
  UserSettingsPage({super.key});

  final formKey = GlobalKey<FormState>();
  final firstNameController = TextEditingController();
  final lastNameController = TextEditingController();
  final mailController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final user = Provider.of<UserProvider>(context).user!;
    firstNameController.text = user.firstName;
    lastNameController.text = user.lastName;
    mailController.text = user.mail;
    return Scaffold(
      appBar: AppBar(
        title: Row(
          children: [
            Icon(Icons.person),
            SizedBox(width: 8),
            const Text('Nutzereinstellungen'),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () async {
          if (formKey.currentState!.validate()) {
            final updatedUser = User(
              id: user.id,
              nfcID: user.nfcID,
              title: user.title,
              firstName: firstNameController.text,
              lastName: lastNameController.text,
              mail: mailController.text,
              station: user.station,
              room: user.room,
              userGroups: user.userGroups,
            );
            final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).updateUser(updatedUser: updatedUser);
            if (wasSuccessful) {
              Navigator.pop(context);
            }
          }
        },
        child: const Icon(Icons.save),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Form(
          key: formKey,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              CustomTextField(
                label: 'Vorname',
                controller: firstNameController,
                validator: Validators.nameValidator,
              ),
              SizedBox(height: 16),
              CustomTextField(
                label: 'Nachname',
                controller: lastNameController,
                validator: Validators.nameValidator,
              ),
              SizedBox(height: 16),
              CustomTextField(
                label: 'E-Mail',
                controller: mailController,
                validator: Validators.mailValidator,
              ),
              SizedBox(height: 16),
              ElevatedButton(
                onPressed: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute<ChangePasswordPage>(
                      builder: (context) => ChangePasswordPage(),
                    ),
                  );
                },
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 16),
                  child: const Text('Passwort ändern'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
