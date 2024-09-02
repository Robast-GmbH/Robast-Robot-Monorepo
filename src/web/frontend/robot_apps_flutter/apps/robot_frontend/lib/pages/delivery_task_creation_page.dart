import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/controller/delivery_time_controller.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/selectors/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:robot_frontend/widgets/selectors/time_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_selector.dart';

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
  final senderTimeController = DeliveryTimeController();
  final recipientTimeController = DeliveryTimeController();

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Lieferauftrag erstellen',
      child: Padding(
        padding: const EdgeInsets.only(left: 64, right: 64, bottom: 32, top: 16),
        child: Column(
          children: [
            Expanded(
              child: ModuleContentCreationView(
                moduleContentController: moduleContentController,
              ),
            ),
            const SizedBox(
              height: 16,
            ),
            RoundedContainer(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: Row(
                  children: [
                    const Text(
                      'Benötigte Größe',
                      textAlign: TextAlign.start,
                      style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
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
                    top: 8,
                  ),
                  child: Text(
                    'Sender',
                    style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
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
                    const SizedBox(
                      width: 8,
                    ),
                    Expanded(
                      child: UserGroupsSelector(
                        controller: senderUserGroupsSelectionController,
                      ),
                    ),
                  ],
                ),
                const SizedBox(
                  height: 8,
                ),
                Row(
                  children: [
                    Expanded(
                      flex: 2,
                      child: LocationSelector(
                        controller: startController,
                        label: 'Start',
                      ),
                    ),
                    const SizedBox(
                      width: 8,
                    ),
                    Expanded(
                      child: TimeSelector(
                        deliveryTimeController: senderTimeController,
                        onTimeSelected: (dateTime) {
                          setState(() {
                            recipientTimeController.value = dateTime;
                          });
                        },
                      ),
                    ),
                  ],
                ),
                const Padding(
                  padding: EdgeInsets.only(
                    top: 8,
                    left: 8,
                  ),
                  child: Text(
                    'Empfänger',
                    style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
                  ),
                ),
                Row(
                  children: [
                    Expanded(
                      child: UserSelector(
                        controller: recipientUserController,
                      ),
                    ),
                    const SizedBox(
                      width: 8,
                    ),
                    Expanded(
                      child: UserGroupsSelector(
                        controller: recipientUserGroupsSelectionController,
                      ),
                    ),
                  ],
                ),
                const SizedBox(
                  height: 8,
                ),
                Row(
                  children: [
                    Expanded(
                      flex: 2,
                      child: LocationSelector(
                        controller: targetController,
                        label: 'Ziel',
                      ),
                    ),
                    const SizedBox(
                      width: 8,
                    ),
                    Expanded(
                      child: TimeSelector(
                        deliveryTimeController: recipientTimeController,
                        onTimeSelected: (datetime) {
                          if (senderTimeController.value != null && datetime.isBefore(senderTimeController.value!)) {
                            recipientTimeController.value = senderTimeController.value;
                          }
                        },
                      ),
                    ),
                  ],
                ),
              ],
            ),
            const SizedBox(
              height: 16,
            ),
            RoundedButton(
              padding: const EdgeInsets.symmetric(horizontal: 4),
              color: Colors.black.withOpacity(0.2),
              onPressed: () async {
                await Provider.of<TaskProvider>(context, listen: false).createDeliveryTaskRequest(
                  requiredSubmoduleType: requiredSubmoduleType,
                  pickupTargetID: startController.room!,
                  pickupEarliestStartTime: senderTimeController.timeAsSecondsSinceEpoch(),
                  senderUserIDs: [
                    if (senderUserController.selectedUser != null) senderUserController.selectedUser!.id,
                  ],
                  senderUserGroups: senderUserGroupsSelectionController.selectionAsStringList(),
                  dropoffTargetID: targetController.room!,
                  dropoffEarliestStartTime: recipientTimeController.timeAsSecondsSinceEpoch(),
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
              child: const Padding(
                padding: EdgeInsets.symmetric(vertical: 8),
                child: Text(
                  'Auftrag erstellen',
                  style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
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
          color: requiredSubmoduleType == type ? RobotColors.accent : Colors.black.withOpacity(0.1),
          border: Border.all(color: Colors.white.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          text,
          textAlign: TextAlign.center,
          style: const TextStyle(
            fontSize: 24,
            color: RobotColors.secondaryText,
          ),
        ),
      ),
    );
  }
}
