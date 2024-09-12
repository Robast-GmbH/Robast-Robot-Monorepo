import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
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
                children: User.userGroupsByDisplayName.entries.map(
                  (entry) {
                    return Row(
                      children: [
                        buildUserGroupSelector(
                          label: entry.value,
                          value: controller.userGroups.contains(entry.key),
                          onChanged: ({required bool newValue}) {
                            if (newValue) {
                              controller.userGroups.add(entry.key);
                            } else {
                              controller.userGroups.remove(entry.key);
                            }
                            setState(() {});
                            widget.onChanged?.call();
                          },
                        ),
                      ],
                    );
                  },
                ).toList(),
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
