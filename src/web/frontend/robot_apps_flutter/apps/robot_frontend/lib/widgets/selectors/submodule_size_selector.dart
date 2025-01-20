import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:shared_data_models/shared_data_models.dart';

class SubmoduleSizeSelector extends StatefulWidget {
  const SubmoduleSizeSelector({required this.controller, super.key});
  final SubmoduleSizeController controller;
  @override
  State<SubmoduleSizeSelector> createState() => _SubmoduleSizeSelectorState();
}

class _SubmoduleSizeSelectorState extends State<SubmoduleSizeSelector> {
  @override
  Widget build(BuildContext context) {
    return Row(
      children: Submodule.sizesByDisplayName.entries.map((entry) {
        return Expanded(
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8.0),
            child: SubmoduleSizeButton(
                onPressed: () {
                  setState(() {
                    widget.controller.size = entry.key;
                  });
                },
                text: entry.value,
                isSelected: widget.controller.size == entry.key),
          ),
        );
      }).toList(),
    );
  }
}

class SubmoduleSizeButton extends StatelessWidget {
  const SubmoduleSizeButton({super.key, required this.onPressed, required this.text, required this.isSelected});
  final VoidCallback onPressed;
  final String text;
  final bool isSelected;
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onPressed,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: isSelected ? RobotColors.accent : Colors.black.withOpacity(0.1),
          border: Border.all(color: Colors.white.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          text,
          textAlign: TextAlign.center,
          style: const TextStyle(
            fontSize: 24,
            color: RobotColors.secondaryText,
          ),
        ),
      ),
    );
  }
}
