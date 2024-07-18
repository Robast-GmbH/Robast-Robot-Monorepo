import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/module_content_view.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';

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

  User? authUser;
  final userGroupsSelectionController = UserGroupsSelectionController();

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
        padding: const EdgeInsets.all(64),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Expanded(
              child: Card(
                color: Colors.white.withOpacity(0.4),
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 20),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Padding(
                        padding: EdgeInsets.only(left: 8, bottom: 8),
                        child: Text('Fracht', style: TextStyle(fontSize: 26)),
                      ),
                      Expanded(
                        child: ModuleContentView(
                          moduleContentController: moduleContentController,
                        ),
                      ),
                    ],
                  ),
                ),
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
              children: [
                buildLabeledLocationSelector(controller: startController, label: 'Start'),
                buildLabeledLocationSelector(controller: targetController, label: 'Ziel'),
                Card(
                  color: Colors.white.withOpacity(0.4),
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                    child: Row(
                      children: [
                        const Text(
                          'Autorisierte Person',
                          textAlign: TextAlign.end,
                          style: TextStyle(
                            fontSize: 24,
                          ),
                        ),
                        const SizedBox(
                          width: 16,
                        ),
                        Expanded(
                          child: FutureBuilder<List<User>>(
                            future: loadUsersFuture,
                            builder: (context, snapshot) {
                              return DropdownButton<User>(
                                disabledHint: const Text(''),
                                isExpanded: true,
                                value: authUser,
                                items: snapshot.connectionState != ConnectionState.done
                                    ? []
                                    : snapshot.data!
                                        .map(
                                          (e) => DropdownMenuItem<User>(
                                            value: e,
                                            child: Text('${e.firstName} ${e.lastName}'),
                                          ),
                                        )
                                        .toList(),
                                onChanged: (value) {
                                  setState(() {
                                    authUser = value;
                                  });
                                },
                              );
                            },
                          ),
                        ),
                        const SizedBox(
                          width: 8,
                        ),
                        IconButton(
                          onPressed: () {
                            setState(() {
                              authUser = null;
                            });
                          },
                          icon: const Icon(Icons.delete),
                        ),
                        const SizedBox(
                          width: 32,
                        ),
                        const Text(
                          'Autorisierte Gruppen',
                          textAlign: TextAlign.end,
                          style: TextStyle(
                            fontSize: 24,
                          ),
                        ),
                        const SizedBox(
                          width: 16,
                        ),
                        UserGroupsSelector(controller: userGroupsSelectionController),
                      ],
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(
              height: 16,
            ),
            CustomButtonView(
              padding: const EdgeInsets.all(16),
              onPressed: () async {
                await Provider.of<TaskProvider>(context, listen: false).createTaskRequest(
                  startID: startController.room,
                  targetID: targetController.room,
                  requiredDrawerType: requiredDrawerType,
                  payload: moduleContentController.createPayload(),
                  authUsers: [
                    if (authUser != null) authUser!.id,
                  ],
                  authUserGroups: userGroupsSelectionController.selectionAsStringList(),
                );
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

  Card buildLabeledLocationSelector({required LocationSelectionController controller, required String label}) {
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            Text(
              label,
              textAlign: TextAlign.end,
              style: const TextStyle(
                fontSize: 24,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: LocationSelector(
                controller: controller,
              ),
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
