import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_view.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';
import 'package:uuid/uuid.dart';
import 'package:uuid/v4.dart';

class DeliveryTaskCreationPage extends StatefulWidget {
  const DeliveryTaskCreationPage({super.key});

  @override
  State<DeliveryTaskCreationPage> createState() => _DeliveryTaskCreationPageState();
}

class _DeliveryTaskCreationPageState extends State<DeliveryTaskCreationPage> {
  final moduleContentController = ModuleContentController();
  int requiredDrawerType = 1;

  final startController = LocationSelectionController();
  final targetController = LocationSelectionController();

  final senderUserController = UserSelectionController();
  final recipientUserController = UserSelectionController();
  final senderUserGroupsSelectionController = UserGroupsSelectionController();
  final recipientUserGroupsSelectionController = UserGroupsSelectionController();

  late Future<List<User>> loadUsersFuture;

  @override
  void initState() {
    super.initState();
    loadUsersFuture = Provider.of<UserProvider>(context, listen: false).getUsers();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Lieferauftrag erstellen',
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 64, vertical: 32),
        child: Column(
          children: [
            Expanded(
              child: ModuleContentView(
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
                            child: buildRequiredDrawerSizeButton(type: 1, text: 'Small'),
                          ),
                          const SizedBox(width: 16),
                          Expanded(
                            child: buildRequiredDrawerSizeButton(type: 2, text: 'Medium'),
                          ),
                          const SizedBox(width: 16),
                          Expanded(
                            child: buildRequiredDrawerSizeButton(type: 3, text: 'Large'),
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
            CustomButtonView(
              padding: const EdgeInsets.all(16),
              onPressed: () async {
                final id = const Uuid().v4();
                final itemsByChange = moduleContentController.createItemsByChange();
                final dropoffItemsByChange = itemsByChange.map((key, value) => MapEntry(key, -value));
                final task = Task.delivery(
                  id: id,
                  requiredDrawerType: requiredDrawerType,
                  pickupTaskID: const Uuid().v4(),
                  pickupTargetID: startController.room!,
                  pickupItemsByChange: itemsByChange,
                  senderUserIDs: [
                    if (senderUserController.selectedUser != null) senderUserController.selectedUser!.id,
                  ],
                  senderUserGroups: senderUserGroupsSelectionController.selectionAsStringList(),
                  dropoffTaskID: const Uuid().v4(),
                  dropoffTargetID: targetController.room!,
                  dropoffItemsByChange: dropoffItemsByChange,
                  recipientUserIDs: [
                    if (recipientUserController.selectedUser != null) recipientUserController.selectedUser!.id,
                  ],
                  recipientUserGroups: recipientUserGroupsSelectionController.selectionAsStringList(),
                );
                print(jsonEncode(task.toJson()));
                // await Provider.of<TaskProvider>(context, listen: false).createTaskRequest(
                //   startID: startController.room,
                //   targetID: targetController.room,
                //   requiredDrawerType: requiredDrawerType,
                //   itemsByChange: moduleContentController.createItemsByChange(),
                //   senderAuthUsers: [
                //     if (senderUserController.selectedUser != null) senderUserController.selectedUser!.id,
                //   ],
                //   senderAuthUserGroups: senderUserGroupsSelectionController.selectionAsStringList(),
                //   recipientAuthUsers: [
                //     if (recipientUserController.selectedUser != null) recipientUserController.selectedUser!.id,
                //   ],
                //   recipientAuthUserGroups: recipientUserGroupsSelectionController.selectionAsStringList(),
                // );
                if (context.mounted) {
                  Navigator.of(context).pop();
                }
              },
              text: 'Auftrag erstellen',
            ),
          ],
        ),
      ),
    );
  }

  GestureDetector buildRequiredDrawerSizeButton({required int type, required String text}) {
    return GestureDetector(
      onTap: () {
        setState(() {
          requiredDrawerType = type;
        });
      },
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: requiredDrawerType == type ? Colors.blue.withOpacity(0.5) : Colors.white.withOpacity(0.4),
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
