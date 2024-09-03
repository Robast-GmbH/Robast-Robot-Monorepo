import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';

import 'package:web_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class UserGroupsSelector extends StatefulWidget {
  const UserGroupsSelector({required this.controller, this.onChanged, super.key});
  final UserGroupsSelectionController controller;
  final void Function()? onChanged;
  @override
  State<UserGroupsSelector> createState() => _UserGroupsSelectorState();
}

class _UserGroupsSelectorState extends State<UserGroupsSelector> {
  @override
  Widget build(BuildContext context) {
    final controller = widget.controller;
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 15),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Text(
              'Autorisierte Gruppen',
              textAlign: TextAlign.start,
              style: TextStyle(fontSize: 24, color: WebColors.secondaryText),
            ),
            SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              child: Row(
                children: [
                  buildUserGroupSelector(
                    label: 'Patient',
                    value: controller.isPatient,
                    onChanged: ({required bool newValue}) {
                      setState(() => controller.isPatient = newValue);
                      widget.onChanged?.call();
                    },
                  ),
                  const SizedBox(
                    width: 8,
                  ),
                  buildUserGroupSelector(
                    label: 'Angestellte/r',
                    value: controller.isStaff,
                    onChanged: ({required bool newValue}) {
                      setState(() => controller.isStaff = newValue);
                      widget.onChanged?.call();
                    },
                  ),
                  const SizedBox(
                    width: 8,
                  ),
                  buildUserGroupSelector(
                    label: 'Admin',
                    value: controller.isAdmin,
                    onChanged: ({required bool newValue}) {
                      setState(() => controller.isAdmin = newValue);
                      widget.onChanged?.call();
                    },
                  ),
                ],
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
            activeColor: WebColors.accent,
            checkColor: WebColors.secondaryIcon,
            value: value,
            onChanged: (newValue) => onChanged(newValue: newValue ?? false),
          ),
          Text(
            label,
            style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
          ),
        ],
      ),
    );
  }
}
