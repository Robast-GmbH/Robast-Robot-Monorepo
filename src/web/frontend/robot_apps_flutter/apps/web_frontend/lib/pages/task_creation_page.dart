import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/controller/delivery_time_controller.dart';
import 'package:web_frontend/models/controller/location_selection_controller.dart';
import 'package:web_frontend/models/controller/module_content_controller.dart';
import 'package:web_frontend/models/controller/submodule_type_controller.dart';
import 'package:web_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:web_frontend/models/controller/user_selection_controller.dart';
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
  final submoduleTypeController = SubmoduleTypeController();

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
        submoduleTypeController.value != null &&
        startController.room != null &&
        targetController.room != null &&
        (senderUserController.selectedUser != null || senderUserGroupsSelectionController.userGroups.isNotEmpty) &&
        (recipientUserController.selectedUser != null || recipientUserGroupsSelectionController.userGroups.isNotEmpty);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Row(
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
          padding: const EdgeInsets.symmetric(horizontal: 8),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              ModuleContentCreationView(
                moduleContentController: moduleContentController,
                label: 'Fracht',
              ),
              SizedBox(
                height: 8,
              ),
              SubmoduleTypeSelector(
                controller: submoduleTypeController,
              ),
              SizedBox(
                height: 16,
              ),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Padding(
                    padding: const EdgeInsets.only(left: 8.0),
                    child: Text(
                      'Sender',
                      style: TextStyle(color: WebColors.primaryText, fontSize: 24),
                    ),
                  ),
                  UserSelector(
                    controller: senderUserController,
                    initWithSessionUser: true,
                  ),
                  SizedBox(
                    height: 8,
                  ),
                  UserGroupsSelector(controller: senderUserGroupsSelectionController),
                  SizedBox(
                    height: 8,
                  ),
                  LocationSelector(controller: startController, label: 'Start'),
                  SizedBox(
                    height: 8,
                  ),
                  TimeSelector(deliveryTimeController: senderTimeController),
                ],
              ),
              SizedBox(
                height: 16,
              ),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Padding(
                    padding: const EdgeInsets.only(left: 8.0),
                    child: Text(
                      'Empfänger',
                      style: TextStyle(color: WebColors.primaryText, fontSize: 24),
                    ),
                  ),
                  UserSelector(
                    controller: recipientUserController,
                  ),
                  SizedBox(
                    height: 8,
                  ),
                  UserGroupsSelector(controller: recipientUserGroupsSelectionController),
                  SizedBox(
                    height: 8,
                  ),
                  LocationSelector(controller: targetController, label: 'Ziel'),
                  SizedBox(
                    height: 8,
                  ),
                  TimeSelector(deliveryTimeController: recipientTimeController),
                  SizedBox(
                    height: 8,
                  ),
                ],
              ),
              SizedBox(
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
                    showDialog(
                        context: context,
                        builder: (
                          context,
                        ) =>
                            NfcMissingDialog(identifiers: identifiers));
                    return;
                  }
                  if (validateInputs()) {
                    await Provider.of<TaskProvider>(context, listen: false).createDeliveryTaskRequest(
                      requiredSubmoduleType: submoduleTypeController.valueAsInt(),
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
                      builder: (context) => InvalidInputsDialog(),
                    );
                  }
                },
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 24),
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
              SizedBox(
                height: 16,
              ),
            ],
          ),
        ),
      ),
    );
  }
}
