import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class TaskCreationPage extends StatelessWidget {
  const TaskCreationPage({super.key});

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
                      onPressed: () {},
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Abholung',
                      onPressed: () {},
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
                      onPressed: () {},
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Inhalte verteilen',
                      onPressed: () {},
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
