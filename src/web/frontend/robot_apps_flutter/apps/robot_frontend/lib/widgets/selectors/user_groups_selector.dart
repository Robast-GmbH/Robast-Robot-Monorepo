import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
        child: Row(
          children: [
            const Text(
              'Autorisierte Gruppen',
              textAlign: TextAlign.end,
              style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
            Expanded(
              child: SingleChildScrollView(
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
                ).toList()),
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
          const SizedBox(
            width: 16,
          ),
          Checkbox(
            activeColor: RobotColors.accent,
            checkColor: RobotColors.secondaryIcon,
            value: value,
            onChanged: (newValue) => onChanged(newValue: newValue ?? false),
          ),
          Text(
            label,
            style: const TextStyle(color: RobotColors.secondaryText, fontSize: 24),
          ),
        ],
      ),
    );
  }
}
