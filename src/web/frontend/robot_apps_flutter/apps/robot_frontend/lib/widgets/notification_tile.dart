import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class NotificationTile extends StatelessWidget {
  const NotificationTile({
    super.key,
    required this.title,
    required this.message,
    required this.onPressed,
    required this.actionIcon,
  });

  final String title;
  final String message;
  final VoidCallback onPressed;
  final IconData actionIcon;

  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            const Padding(
              padding: EdgeInsets.only(
                right: 32,
                left: 16,
              ),
              child: Icon(
                Icons.warning,
                color: RobotColors.error,
                size: 64,
              ),
            ),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: const TextStyle(color: RobotColors.primaryText, fontSize: 48),
                ),
                Text(
                  message,
                  style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                ),
              ],
            ),
            const Spacer(),
            IconButton(onPressed: onPressed, icon: Icon(actionIcon, color: RobotColors.primaryText, size: 64)),
          ],
        ),
      ),
    );
  }
}
