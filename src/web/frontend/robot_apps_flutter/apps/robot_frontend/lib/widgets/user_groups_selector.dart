import 'package:flutter/material.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';

class UserGroupsSelector extends StatefulWidget {
  const UserGroupsSelector({required this.controller, super.key});
  final UserGroupsSelectionController controller;
  @override
  State<UserGroupsSelector> createState() => _UserGroupsSelectorState();
}

class _UserGroupsSelectorState extends State<UserGroupsSelector> {
  @override
  Widget build(BuildContext context) {
    final controller = widget.controller;
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 15),
        child: Row(
          children: [
            const Text(
              'Autorisierte Gruppen',
              textAlign: TextAlign.end,
              style: TextStyle(
                fontSize: 24,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: Row(
                  children: [
                    buildUserGroupSelector(
                      label: 'Patient',
                      value: controller.isPatient,
                      onChanged: ({required bool newValue}) => setState(() => controller.isPatient = newValue),
                    ),
                    const SizedBox(
                      width: 16,
                    ),
                    buildUserGroupSelector(
                      label: 'Angestellte/r',
                      value: controller.isStaff,
                      onChanged: ({required bool newValue}) => setState(() => controller.isStaff = newValue),
                    ),
                    const SizedBox(
                      width: 16,
                    ),
                    buildUserGroupSelector(
                      label: 'Admin',
                      value: controller.isAdmin,
                      onChanged: ({required bool newValue}) => setState(() => controller.isAdmin = newValue),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget buildUserGroupSelector({required String label, required bool value, required void Function({required bool newValue}) onChanged}) {
    return InkWell(
      onTap: () => onChanged(newValue: !value),
      child: Row(
        children: [
          Checkbox(
            value: value,
            onChanged: (newValue) => onChanged(newValue: newValue ?? false),
          ),
          Text(
            label,
            style: const TextStyle(color: Colors.white70, fontSize: 24),
          ),
        ],
      ),
    );
  }
}
