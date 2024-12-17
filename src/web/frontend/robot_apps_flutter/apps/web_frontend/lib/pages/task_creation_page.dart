import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/widgets/invalid_inputs_dialog.dart';
import 'package:web_frontend/widgets/module_content_creation_view.dart';
import 'package:web_frontend/widgets/nfc_missing_dialog.dart';
import 'package:web_frontend/widgets/selectors/location_selector.dart';
import 'package:web_frontend/widgets/selectors/submodule_type_selector.dart';
import 'package:web_frontend/widgets/selectors/time_selector.dart';
import 'package:web_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:web_frontend/widgets/selectors/user_selector.dart';

class TaskCreationPage extends StatefulWidget {
  const TaskCreationPage({super.key});

  @override
  State<TaskCreationPage> createState() => _TaskCreationPageState();
}

class _TaskCreationPageState extends State<TaskCreationPage> {
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
    return Scaffold(
      appBar: AppBar(
        title: const Row(
          children: [
            Icon(Icons.add_task),
            SizedBox(
              width: 8,
            ),
            Text('Lieferauftrag erstellen'),
          ],
        ),
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              ModuleContentCreationView(
                moduleContentController: moduleContentController,
              ),
              const SizedBox(
                height: 16,
              ),
              SubmoduleTypeSelector(
                controller: submoduleSizeController,
              ),
              const SizedBox(
                height: 16,
              ),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Padding(
                    padding: EdgeInsets.only(left: 8, bottom: 4),
                    child: Text(
                      'Sender',
                      style: TextStyle(color: WebColors.primaryText, fontSize: 24),
                    ),
                  ),
                  UserSelector(
                    controller: senderUserController,
                    initWithSessionUser: true,
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                  UserGroupsSelector(controller: senderUserGroupsSelectionController),
                  const SizedBox(
                    height: 16,
                  ),
                  LocationSelector(controller: startController, label: 'Start'),
                  const SizedBox(
                    height: 16,
                  ),
                  TimeSelector(deliveryTimeController: senderTimeController),
                ],
              ),
              const SizedBox(
                height: 16,
              ),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Padding(
                    padding: EdgeInsets.only(left: 8, bottom: 4),
                    child: Text(
                      'Empfänger',
                      style: TextStyle(color: WebColors.primaryText, fontSize: 24),
                    ),
                  ),
                  UserSelector(
                    controller: recipientUserController,
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                  UserGroupsSelector(controller: recipientUserGroupsSelectionController),
                  const SizedBox(
                    height: 16,
                  ),
                  LocationSelector(controller: targetController, label: 'Ziel'),
                  const SizedBox(
                    height: 16,
                  ),
                  TimeSelector(deliveryTimeController: recipientTimeController),
                  const SizedBox(
                    height: 16,
                  ),
                ],
              ),
              const SizedBox(
                height: 16,
              ),
              ElevatedButton(
                onPressed: () async {
                  if (!(senderUserController.selectedUser?.hasNfcID() ?? true) || (!(recipientUserController.selectedUser?.hasNfcID() ?? true))) {
                    final identifiers = <String>[];
                    if (!(senderUserController.selectedUser?.hasNfcID() ?? true)) {
                      identifiers.add('Sender');
                    }
                    if (!(recipientUserController.selectedUser?.hasNfcID() ?? true)) {
                      identifiers.add('Empfänger');
                    }
                    await showDialog<NfcMissingDialog>(
                      context: context,
                      builder: (
                        context,
                      ) =>
                          NfcMissingDialog(identifiers: identifiers),
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
                    await showDialog<InvalidInputsDialog>(
                      context: context,
                      builder: (context) => const InvalidInputsDialog(),
                    );
                  }
                },
                child: const Padding(
                  padding: EdgeInsets.symmetric(vertical: 24),
                  child: SizedBox(
                    width: double.infinity,
                    child: Text(
                      'Erstellen',
                      textAlign: TextAlign.center,
                      style: TextStyle(fontSize: 24),
                    ),
                  ),
                ),
              ),
              const SizedBox(
                height: 16,
              ),
            ],
          ),
        ),
      ),
    );
  }
}
