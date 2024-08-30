import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class CustomElevatedButton extends StatelessWidget {
  const CustomElevatedButton({this.onPressed, this.label, this.enabled = true, super.key});

  final VoidCallback? onPressed;
  final String? label;
  final bool enabled;
  @override
  Widget build(BuildContext context) {
    return Opacity(
      opacity: enabled ? 1 : 0.3,
      child: ElevatedButton(
        style: ButtonStyle(
          backgroundColor: WidgetStateProperty.all(RobotColors.accent),
        ),
        onPressed: enabled ? onPressed : null,
        child: Padding(
          padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 16),
          child: Text(
            label ?? '',
            style: const TextStyle(fontSize: 40, color: RobotColors.primaryText),
          ),
        ),
      ),
    );
  }
}
