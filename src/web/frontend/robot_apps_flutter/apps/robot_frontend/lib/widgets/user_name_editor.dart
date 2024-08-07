import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/user_name_controller.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

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
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 6, horizontal: 16),
        child: Row(
          children: [
            Expanded(
              flex: 2,
              child: DropdownButton<String>(
                isExpanded: true,
                value: widget.controller.title,
                underline: const SizedBox(),
                onChanged: (value) => setState(() => widget.controller.title = value ?? ''),
                items: Provider.of<UserProvider>(context, listen: false)
                    .availableTitles
                    .map(
                      (title) => DropdownMenuItem<String>(
                        value: title,
                        alignment: Alignment.center,
                        child: Text(title, style: const TextStyle(fontSize: 24)),
                      ),
                    )
                    .toList(),
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              flex: 3,
              child: TextField(
                controller: firstNameController,
                onChanged: (value) => widget.controller.firstName = value,
                style: const TextStyle(fontSize: 24),
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              flex: 3,
              child: TextField(
                controller: lastNameController,
                onChanged: (value) => widget.controller.lastName = value,
                style: const TextStyle(fontSize: 24),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
