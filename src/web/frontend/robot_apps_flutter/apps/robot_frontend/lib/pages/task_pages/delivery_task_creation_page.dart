import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/dialogs/nfc_missing_dialog.dart';
import 'package:robot_frontend/widgets/selectors/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:robot_frontend/widgets/selectors/submodule_size_selector.dart';
import 'package:robot_frontend/widgets/selectors/time_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_selector.dart';
import 'package:shared_data_models/shared_data_models.dart';

class DeliveryTaskCreationPage extends StatefulWidget {
  const DeliveryTaskCreationPage({super.key});

  @override
  State<DeliveryTaskCreationPage> createState() => _DeliveryTaskCreationPageState();
}

class _DeliveryTaskCreationPageState extends State<DeliveryTaskCreationPage> {
  final moduleContentController = ModuleContentController();
  final submoduleSizeController = SubmoduleSizeController();

  final startController = LocationSelectionController();
  final targetController = LocationSelectionController();

  final senderUserController = UserSelectionController();
  final recipientUserController = UserSelectionController();
  final senderUserGroupsSelectionController = UserGroupsSelectionController();
  final recipientUserGroupsSelectionController = UserGroupsSelectionController();
  final senderTimeController = DeliveryTimeController();
  final recipientTimeController = DeliveryTimeController();

  bool validateInputs() {
    return moduleContentController.didItemsChange() &&
        startController.room != null &&
        targetController.room != null &&
        (senderUserController.selectedUser != null || senderUserGroupsSelectionController.userGroups.isNotEmpty) &&
        (recipientUserController.selectedUser != null || recipientUserGroupsSelectionController.userGroups.isNotEmpty);
  }

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
                padding: const EdgeInsets.only(left: 16, right: 8, top: 8, bottom: 8),
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
                    Expanded(flex: 2, child: SubmoduleSizeSelector(controller: submoduleSizeController)),
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
                        onChanged: () {
                          startController.setStation(senderUserController.selectedUser?.station);
                          startController.setRoom(senderUserController.selectedUser?.room);
                          setState(() {});
                        },
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
                        onChanged: () {
                          targetController.setStation(recipientUserController.selectedUser?.station);
                          targetController.setRoom(recipientUserController.selectedUser?.room);
                          setState(() {});
                        },
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
                if (!(senderUserController.selectedUser?.hasNfcID() ?? true) || !(recipientUserController.selectedUser?.hasNfcID() ?? true)) {
                  final identifiers = <String>[];
                  if (!(senderUserController.selectedUser?.hasNfcID() ?? true)) {
                    identifiers.add('${senderUserController.selectedUser!.firstName} ${senderUserController.selectedUser!.lastName}');
                  }
                  if (!(recipientUserController.selectedUser?.hasNfcID() ?? true)) {
                    identifiers.add('${recipientUserController.selectedUser!.firstName} ${recipientUserController.selectedUser!.lastName}');
                  }
                  showDialog(
                    context: context,
                    builder: (context) => NfcMissingDialog(
                      identifiers: identifiers,
                    ),
                  );
                  return;
                }

                if (validateInputs()) {
                  await Provider.of<TaskProvider>(context, listen: false).createDeliveryTaskRequest(
                    requiredSubmoduleType: submoduleSizeController.size,
                    pickupTargetID: startController.room!,
                    pickupEarliestStartTime: senderTimeController.timeAsSecondsSinceEpoch(),
                    senderUserIDs: [
                      if (senderUserController.selectedUser != null) senderUserController.selectedUser!.id,
                    ],
                    senderUserGroups: senderUserGroupsSelectionController.userGroups,
                    dropoffTargetID: targetController.room!,
                    dropoffEarliestStartTime: recipientTimeController.timeAsSecondsSinceEpoch(),
                    recipientUserIDs: [
                      if (recipientUserController.selectedUser != null) recipientUserController.selectedUser!.id,
                    ],
                    recipientUserGroups: recipientUserGroupsSelectionController.userGroups,
                    itemsByChange: moduleContentController.createItemsByChange(),
                  );
                  if (context.mounted) {
                    Navigator.of(context).pop();
                  }
                } else {
                  showDialog(
                    context: context,
                    builder: (context) => AlertDialog(
                      backgroundColor: RobotColors.secondaryBackground,
                      title: const Text(
                        'Ungültige Eingaben',
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 28),
                      ),
                      content: const Text(
                        'Bitte überprüfen Sie ihre Eingaben.',
                        style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
                      ),
                      actions: [
                        TextButton(
                          onPressed: () {
                            Navigator.of(context).pop();
                          },
                          child: const Text('OK', style: TextStyle(color: RobotColors.secondaryText, fontSize: 24)),
                        ),
                      ],
                    ),
                  );
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
}
