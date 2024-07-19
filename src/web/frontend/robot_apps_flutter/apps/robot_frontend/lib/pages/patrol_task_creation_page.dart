import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/location_selector.dart';

class PatrolTaskCreationPage extends StatefulWidget {
  const PatrolTaskCreationPage({super.key});

  @override
  State<PatrolTaskCreationPage> createState() => _PatrolTaskCreationPageState();
}

class _PatrolTaskCreationPageState extends State<PatrolTaskCreationPage> {
  final controller = LocationSelectionController();
  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Zielort anfahren',
      child: Padding(
        padding: const EdgeInsets.all(128),
        child: Column(
          children: [
            const Text(
              'Hier können Sie einen Auftrag erstellen, bei dem der Roboter einen bestimmten Zielort anfährt.',
              style: TextStyle(fontSize: 24),
            ),
            const SizedBox(
              height: 32,
            ),
            LocationSelector(
              controller: controller,
              label: 'Zielort',
            ),
            const SizedBox(
              height: 32,
            ),
            CustomButtonView(
              text: 'Start',
              onPressed: () {},
              padding: const EdgeInsets.all(8),
            ),
          ],
        ),
      ),
    );
  }
}
