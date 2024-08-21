import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';

class DeliveryTaskCreationPage extends StatefulWidget {
  const DeliveryTaskCreationPage({super.key});

  @override
  State<DeliveryTaskCreationPage> createState() => _DeliveryTaskCreationPageState();
}

class _DeliveryTaskCreationPageState extends State<DeliveryTaskCreationPage> {
  final moduleContentController = ModuleContentController();
  int requiredSubmoduleType = 1;

  final startController = LocationSelectionController();
  final targetController = LocationSelectionController();

  final senderUserController = UserSelectionController();
  final recipientUserController = UserSelectionController();
  final senderUserGroupsSelectionController = UserGroupsSelectionController();
  final recipientUserGroupsSelectionController = UserGroupsSelectionController();

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Lieferauftrag erstellen',
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 64, vertical: 32),
        child: Column(
          children: [
            Expanded(
              child: ModuleContentCreationView(
                moduleContentController: moduleContentController,
              ),
            ),
            Card(
              color: Colors.white.withOpacity(0.4),
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: Row(
                  children: [
                    const Text(
                      'Benötigte Größe',
                      textAlign: TextAlign.start,
                      style: TextStyle(
                        fontSize: 24,
                      ),
                    ),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      flex: 2,
                      child: Row(
                        children: [
                          Expanded(
                            child: buildRequiredSubmoduleTypeButton(type: 1, text: 'Small'),
                          ),
                          const SizedBox(width: 16),
                          Expanded(
                            child: buildRequiredSubmoduleTypeButton(type: 2, text: 'Medium'),
                          ),
                          const SizedBox(width: 16),
                          Expanded(
                            child: buildRequiredSubmoduleTypeButton(type: 3, text: 'Large'),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Padding(
                  padding: EdgeInsets.only(
                    left: 8,
                    top: 4,
                  ),
                  child: Text(
                    'Sender',
                    style: TextStyle(fontSize: 28),
                  ),
                ),
                Row(
                  children: [
                    Expanded(
                      child: UserSelector(
                        controller: senderUserController,
                        initWithSessionUser: true,
                      ),
                    ),
                    Expanded(
                      child: UserGroupsSelector(
                        controller: senderUserGroupsSelectionController,
                      ),
                    ),
                  ],
                ),
                LocationSelector(
                  controller: startController,
                  label: 'Start',
                ),
                const Padding(
                  padding: EdgeInsets.only(
                    top: 4,
                    left: 8,
                  ),
                  child: Text(
                    'Empfänger',
                    style: TextStyle(fontSize: 28),
                  ),
                ),
                Row(
                  children: [
                    Expanded(
                      child: UserSelector(
                        controller: recipientUserController,
                      ),
                    ),
                    Expanded(
                      child: UserGroupsSelector(
                        controller: recipientUserGroupsSelectionController,
                      ),
                    ),
                  ],
                ),
                LocationSelector(
                  controller: targetController,
                  label: 'Ziel',
                ),
              ],
            ),
            const SizedBox(
              height: 16,
            ),
            RoundedButton(
              padding: EdgeInsets.symmetric(horizontal: 4),
              color: Colors.white.withOpacity(0.5),
              onPressed: () async {
                await Provider.of<TaskProvider>(context, listen: false).createDeliveryTaskRequest(
                  requiredSubmoduleType: requiredSubmoduleType,
                  pickupTargetID: startController.room!,
                  senderUserIDs: [
                    if (senderUserController.selectedUser != null) senderUserController.selectedUser!.id,
                  ],
                  senderUserGroups: senderUserGroupsSelectionController.selectionAsStringList(),
                  dropoffTargetID: targetController.room!,
                  recipientUserIDs: [
                    if (recipientUserController.selectedUser != null) recipientUserController.selectedUser!.id,
                  ],
                  recipientUserGroups: recipientUserGroupsSelectionController.selectionAsStringList(),
                  itemsByChange: moduleContentController.createItemsByChange(),
                );
                if (context.mounted) {
                  Navigator.of(context).pop();
                }
              },
              child: Padding(
                padding: const EdgeInsets.symmetric(vertical: 8),
                child: Text(
                  'Auftrag erstellen',
                  style: TextStyle(fontSize: 40, color: Colors.white),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  GestureDetector buildRequiredSubmoduleTypeButton({required int type, required String text}) {
    return GestureDetector(
      onTap: () {
        setState(() {
          requiredSubmoduleType = type;
        });
      },
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: requiredSubmoduleType == type ? Colors.blue.withOpacity(0.5) : Colors.white.withOpacity(0.4),
          border: Border.all(color: Colors.white.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          text,
          textAlign: TextAlign.center,
          style: const TextStyle(
            fontSize: 24,
          ),
        ),
      ),
    );
  }
}
