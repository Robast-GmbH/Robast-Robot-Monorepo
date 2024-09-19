import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/selectors/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';
import 'package:robot_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_selector.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
              child: ModuleContentCreationView(
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
                ),
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
