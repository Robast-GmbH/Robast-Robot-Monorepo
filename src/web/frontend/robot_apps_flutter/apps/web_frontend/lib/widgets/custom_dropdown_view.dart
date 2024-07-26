import 'package:flutter/material.dart';

class CustomDropdownView extends StatelessWidget {
  const CustomDropdownView({
    required this.value, required this.items, required this.hint, required this.onChanged, super.key,
  });

  final String? value;
  final List<String> items;
  final String hint;
  final void Function(String?) onChanged;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: DropdownButton<String>(
        isExpanded: true,
        value: value,
        items: items.map((String value) {
          return DropdownMenuItem<String>(
            value: value,
            child: Text(value),
          );
        }).toList(),
        hint: Text(hint),
        onChanged: onChanged,
      ),
    );
  }
}
