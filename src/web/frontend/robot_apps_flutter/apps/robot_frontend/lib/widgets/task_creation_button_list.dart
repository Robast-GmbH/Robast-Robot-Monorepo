import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/pages/task_pages/content_distribution_task_creation_page.dart';
import 'package:robot_frontend/pages/task_pages/delivery_task_creation_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';

class TaskCreationButtonList extends StatelessWidget {
  const TaskCreationButtonList({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      text: 'Auftrag erstellen',
      onPressed: () {},
      content: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const SizedBox(height: 16),
          Expanded(
            child: RoundedButton(
              onPressed: () {
                Navigator.push(context, MaterialPageRoute<DeliveryTaskCreationPage>(builder: (context) => const DeliveryTaskCreationPage()));
              },
              child: const Text(
                'Abholen und Zustellen',
                style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
              ),
            ),
          ),
          const SizedBox(height: 16),
          Expanded(
            child: RoundedButton(
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute<ContentDistributionTaskCreationPage>(builder: (context) => const ContentDistributionTaskCreationPage()),
                );
              },
              child: const Text(
                'Beladen und Zustellen',
                style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
              ),
            ),
          ),
          const SizedBox(height: 16),
          Expanded(
            child: RoundedButton(
              onPressed: () {},
              child: const Text(
                'Benutzerdefiniert',
                style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
