import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/pages/setting_pages/cleaning_page.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class NotificationTile extends StatelessWidget {
  const NotificationTile({super.key});

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
            const Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Reinigung erforderlich',
                  style: TextStyle(color: RobotColors.primaryText, fontSize: 48),
                ),
                Text(
                  'Bitte reinigen Sie den Roboter',
                  style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                ),
              ],
            ),
            const Spacer(),
            IconButton(
                onPressed: () {
                  Navigator.push(context, MaterialPageRoute<void>(builder: (context) => const CleaningPage()));
                },
                icon: const Icon(Icons.arrow_right_alt, color: RobotColors.primaryText, size: 64)),
          ],
        ),
      ),
    );
  }
}
