import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_dropdown_button.dart';
import 'package:robot_frontend/widgets/custom_textfield.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class UserNameEditor extends StatefulWidget {
  const UserNameEditor({
    required this.controller,
    super.key,
  });

  final UserNameController controller;

  @override
  State<UserNameEditor> createState() => _UserNameEditorState();
}

class _UserNameEditorState extends State<UserNameEditor> {
  late final TextEditingController firstNameController;
  late final TextEditingController lastNameController;

  @override
  void initState() {
    super.initState();
    firstNameController = TextEditingController(text: widget.controller.firstName);
    lastNameController = TextEditingController(text: widget.controller.lastName);
  }

  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 6, horizontal: 16),
        child: Row(
          children: [
            const Text(
              'Name',
              style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              flex: 2,
              child: CustomDropdownButton(
                value: widget.controller.title,
                onChanged: (value) => setState(() => widget.controller.title = value ?? ''),
                items: Provider.of<UserProvider>(context, listen: false).availableTitles.toList(),
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              flex: 3,
              child: CustomTextfield(
                controller: firstNameController,
                onChanged: (value) => widget.controller.firstName = value,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              flex: 3,
              child: CustomTextfield(
                controller: lastNameController,
                onChanged: (value) => widget.controller.lastName = value,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
