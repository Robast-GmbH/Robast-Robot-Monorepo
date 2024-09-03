import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';

class CustomDropdownButton extends StatelessWidget {
  const CustomDropdownButton({
    required this.value,
    required this.items,
    required this.onChanged,
    this.disabledHint,
    this.hint,
    super.key,
  });

  final String? value;
  final List<String> items;
  final void Function(String?) onChanged;
  final String? disabledHint;
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
              style: const TextStyle(fontSize: 18, color: WebColors.secondaryText),
            )
          : null,
      disabledHint: Text(
        disabledHint ?? '',
        style: const TextStyle(fontSize: 18, color: WebColors.secondaryIcon),
      ),
      onChanged: onChanged,
      items: items
          .map(
            (item) => DropdownMenuItem<String>(
              value: item,
              alignment: Alignment.centerLeft,
              child: Text(
                item,
                style: const TextStyle(fontSize: 18, color: WebColors.secondaryIcon),
              ),
            ),
          )
          .toList(),
    );
  }
}
