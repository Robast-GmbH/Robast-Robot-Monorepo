import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/clock_view.dart';

class StatusBar extends StatelessWidget {
  const StatusBar({
    this.title,
    this.onBackButtonPressed,
    super.key,
  });

  final VoidCallback? onBackButtonPressed;
  final String? title;

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        if (onBackButtonPressed != null) ...[
          Align(
            alignment: Alignment.topLeft,
            child: IconButton(
              iconSize: 48,
              icon: const Icon(
                Icons.arrow_back,
                color: RobotColors.primaryIcon,
              ),
              onPressed: onBackButtonPressed,
            ),
          ),
        ],
        if (title != null)
          Align(
            alignment: Alignment.topCenter,
            child: Padding(
              padding: const EdgeInsets.only(top: 8),
              child: Text(
                title!,
                style: const TextStyle(
                  color: RobotColors.primaryText,
                  fontSize: 32,
                ),
              ),
            ),
          ),
        const Align(
          alignment: Alignment.topRight,
          child: Padding(
            padding: EdgeInsets.only(right: 16, top: 8),
            child: ClockView(),
          ),
        ),
      ],
    );
  }
}
