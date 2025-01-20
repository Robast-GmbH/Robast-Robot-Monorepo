import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/custom_textfield.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class UserCreationPage extends StatelessWidget {
  UserCreationPage({super.key});

  final firstNameController = CustomFocusNode(key: GlobalKey(), text: '');
  final lastNameController = CustomFocusNode(key: GlobalKey(), text: '');

  @override
  Widget build(BuildContext context) {
    firstNameController.next = lastNameController;
    return CustomScaffold(
      title: 'User erstellen',
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              children: [
                Expanded(
                  child: RoundedContainer(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                      child: Row(
                        children: [
                          const Text(
                            "Vorname:",
                            style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
                          ),
                          const SizedBox(
                            width: 16,
                          ),
                          Expanded(
                              child: CustomTextfield(
                            focusNode: firstNameController,
                          )),
                        ],
                      ),
                    ),
                  ),
                ),
                const SizedBox(
                  width: 16,
                ),
                Expanded(
                  child: RoundedContainer(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                      child: Row(
                        children: [
                          const Text(
                            "Nachname:",
                            style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
                          ),
                          const SizedBox(
                            width: 16,
                          ),
                          Expanded(
                              child: CustomTextfield(
                            focusNode: lastNameController,
                          )),
                        ],
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const Expanded(child: SizedBox()),
            RoundedButton(
              onPressed: () async {
                await Provider.of<UserProvider>(context, listen: false).createUser(
                  newUser: User(
                    id: '',
                    nfcID: null,
                    mail: null,
                    title: '',
                    firstName: firstNameController.text,
                    lastName: lastNameController.text,
                    station: '',
                    room: '',
                    userGroups: [],
                  ),
                );
                if (!context.mounted) return;
                Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                Navigator.pop(context);
              },
              child: const Padding(
                padding: EdgeInsets.symmetric(horizontal: 16, vertical: 16),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Text(
                      "Erstellen",
                      style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
                    ),
                  ],
                ),
              ),
            ),
            SizedBox(
              height: Provider.of<KeyboardProvider>(context).focusNode != null ? 400 : 16,
            ),
          ],
        ),
      ),
    );
  }
}
