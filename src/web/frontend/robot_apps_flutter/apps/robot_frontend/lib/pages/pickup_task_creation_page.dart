import 'package:flutter/material.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_view.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';

class PickupTaskCreationPage extends StatefulWidget {
  const PickupTaskCreationPage({super.key});

  @override
  State<PickupTaskCreationPage> createState() => _PickupTaskCreationPageState();
}

class _PickupTaskCreationPageState extends State<PickupTaskCreationPage> {
  final moduleContentController = ModuleContentController();
  final locationSelectionController = LocationSelectionController();
  final userSelectionController = UserSelectionController();
  final userGroupsSelectionController = UserGroupsSelectionController();
  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Abholung',
      child: Padding(
        padding: const EdgeInsets.all(128),
        child: Column(
          children: [
            Expanded(
              child: ModuleContentView(
                moduleContentController: moduleContentController,
              ),
            ),
            const SizedBox(
              height: 32,
            ),
            LocationSelector(
              controller: locationSelectionController,
              label: 'Abholort',
            ),
            const SizedBox(
              height: 8,
            ),
            Row(
              children: [
                Expanded(
                  child: UserSelector(
                    controller: userSelectionController,
                  ),
                ),
                Expanded(
                  child: UserGroupsSelector(
                    controller: userGroupsSelectionController,
                  ),
                )
              ],
            ),
            const SizedBox(
              height: 16,
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
