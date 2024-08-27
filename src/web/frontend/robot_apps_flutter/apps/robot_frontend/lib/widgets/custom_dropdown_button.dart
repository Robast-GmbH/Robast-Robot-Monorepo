import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class CustomDropdownButton extends StatelessWidget {
  const CustomDropdownButton({
    required this.value,
    required this.items,
    required this.onChanged,
    this.hint,
    super.key,
  });

  final String? value;
  final List<String> items;
  final void Function(String?) onChanged;
  final String? hint;

  @override
  Widget build(BuildContext context) {
    return DropdownButtonFormField<String>(
      isExpanded: true,
      isDense: false,
      decoration: const InputDecoration(contentPadding: EdgeInsets.all(0)),
      value: value,
      hint: hint != null
          ? Text(
              hint!,
              style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            )
          : null,
      disabledHint: Text(
        value ?? '',
        style: const TextStyle(fontSize: 24, color: RobotColors.secondaryIcon),
      ),
      onChanged: onChanged,
      items: items
          .map(
            (item) => DropdownMenuItem<String>(
              value: item,
              alignment: Alignment.center,
              child: Text(
                item,
                style: const TextStyle(fontSize: 24, color: RobotColors.secondaryIcon),
              ),
            ),
          )
          .toList(),
    );
  }
}
