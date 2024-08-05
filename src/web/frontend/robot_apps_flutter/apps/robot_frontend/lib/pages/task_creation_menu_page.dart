import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/content_distribution_task_creation_page.dart';
import 'package:robot_frontend/pages/delivery_task_creation_page.dart';
import 'package:robot_frontend/pages/patrol_task_creation_page.dart';
import 'package:robot_frontend/pages/pickup_task_creation_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class TaskCreationMenuPage extends StatelessWidget {
  const TaskCreationMenuPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Auftrag erstellen',
      child: Padding(
        padding: const EdgeInsets.all(128),
        child: Row(
          children: [
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Lieferung',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<DeliveryTaskCreationPage>(
                          builder: (context) => const DeliveryTaskCreationPage(),
                        ),
                      ),
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Abholung',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<PickupTaskCreationPage>(
                          builder: (context) => const PickupTaskCreationPage(),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Zielort anfahren',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<PatrolTaskCreationPage>(
                          builder: (context) => const PatrolTaskCreationPage(),
                        ),
                      ),
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Inhalte verteilen',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<ContentDistributionTaskCreationPage>(
                          builder: (context) => const ContentDistributionTaskCreationPage(),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
