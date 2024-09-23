import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class UserCreationPage extends StatelessWidget {
  UserCreationPage({super.key});

  final formKey = GlobalKey<FormState>();
  final firstNameController = TextEditingController();
  final lastNameController = TextEditingController();
  final mailController = TextEditingController();
  final groupsController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Row(
          children: [
            Icon(Icons.person_add),
            SizedBox(
              width: 16,
            ),
            Text('Nutzererstellung')
          ],
        ),
      ),
      body: Form(
        key: formKey,
        child: Padding(
          padding: const EdgeInsets.all(16),
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
              CustomTextField(
                label: 'Nutzergruppen',
                controller: groupsController,
                validator: Validators.userGroupsValidator,
              ),
              SizedBox(height: 16),
              ElevatedButton(
                onPressed: () async {
                  if (formKey.currentState!.validate()) {
                    final wasUserCreated = await Provider.of<UserProvider>(context, listen: false).createUser(
                      newUser: User(
                        id: '',
                        nfcID: '',
                        mail: mailController.text,
                        title: '',
                        firstName: firstNameController.text,
                        lastName: lastNameController.text,
                        station: '',
                        room: '',
                        userGroups: groupsController.text.split(','),
                      ),
                    );
                    if (wasUserCreated) {
                      Navigator.pop(context, true);
                    }
                  }
                },
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 16),
                  child: const Text('Nutzer erstellen'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
