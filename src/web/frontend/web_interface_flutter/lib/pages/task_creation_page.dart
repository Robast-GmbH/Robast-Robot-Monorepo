import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';
import 'package:web_interface_flutter/widgets/robo_map.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';
import 'package:web_interface_flutter/widgets/rounded_button.dart';

enum Tabs { targetSelection, moduleSelection, detailsInput }

class TaskCreationPage extends StatefulWidget {
  const TaskCreationPage({super.key});

  @override
  State<TaskCreationPage> createState() => _TaskCreationPageState();
}

class _TaskCreationPageState extends State<TaskCreationPage> {
  Tabs currentTab = Tabs.targetSelection;
  final ownerController = TextEditingController();
  final recipientController = TextEditingController();
  final subjectController = TextEditingController();
  final mapController = MapController();
  int? selectedModuleID;

  @override
  Widget build(BuildContext context) {
    return PageFrame(
      title: "Neuer Auftrag",
      color: AppColors.green,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(children: [
          buildTab(title: "1. Zielauswahl", child: buildTargetSelection(), tab: Tabs.targetSelection),
          const SizedBox(
            width: 16,
          ),
          buildTab(title: "2. Modulauswahl", child: buildModuleSelection(), tab: Tabs.moduleSelection),
          const SizedBox(
            width: 16,
          ),
          buildTab(title: "3. Details", child: buildDetailsInput(), tab: Tabs.detailsInput)
        ]),
      ),
    );
  }

  Widget buildOptionalExpanded({required bool shouldExpand, required Widget child}) {
    return shouldExpand
        ? Expanded(
            child: child,
          )
        : child;
  }

  Widget buildTab({required String title, required Widget child, required Tabs tab}) {
    return buildOptionalExpanded(
      shouldExpand: currentTab == tab,
      child: GestureDetector(
        onTap: () {
          currentTab = tab;
          setState(() {});
        },
        child: RotatedBox(
            quarterTurns: currentTab == tab ? 0 : 1,
            child: Container(
              width: currentTab != tab ? double.infinity : null,
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: AppColors.lightGrey,
                borderRadius: BorderRadius.circular(16),
              ),
              child: Stack(
                fit: currentTab == tab ? StackFit.expand : StackFit.loose,
                children: [
                  Text(
                    title,
                    style: const TextStyle(fontSize: 18),
                  ),
                  currentTab == tab ? child : const SizedBox(),
                ],
              ),
            )),
      ),
    );
  }

  Widget buildTargetSelection() {
    return Padding(
      padding: const EdgeInsets.all(32),
      child: RoboMap(controller: mapController),
    );
  }

  Widget buildModuleSelection() {
    return Padding(
      padding: const EdgeInsets.all(32),
      child: RobotClone(
        selectedModule: selectedModuleID,
        onPressed: (moduleID) {
          if (selectedModuleID == moduleID) {
            selectedModuleID = null;
          } else {
            selectedModuleID = moduleID;
          }

          setState(() {});
        },
      ),
    );
  }

  Widget buildDetailsInput() {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.end,
        children: [
          const SizedBox(
            height: 48,
          ),
          buildDetailsInputField(hintText: "Auftraggeber", controller: ownerController),
          const SizedBox(
            height: 8,
          ),
          buildDetailsInputField(hintText: "Empfänger", controller: recipientController),
          const SizedBox(
            height: 8,
          ),
          Expanded(child: buildDetailsInputField(hintText: "Betreff", controller: subjectController)),
          const SizedBox(
            height: 8,
          ),
          RoundedButton(
            text: "Auftrag bestätigen",
            color: AppColors.green,
            borderRadius: BorderRadius.circular(8),
            onTap: () async {
              if (ownerController.text.isEmpty || recipientController.text.isEmpty || selectedModuleID == null || mapController.position == null) {
                await showDialog(
                  context: context,
                  builder: (context) => AlertDialog(
                    title: const Text("Angaben unkomplett"),
                    content: const Text("Bitte alle Felder ausfüllen, um Aufgabe zu erstellen."),
                    actions: [
                      TextButton(
                        onPressed: () {
                          Navigator.pop(context);
                        },
                        child: const Text("Okay"),
                      )
                    ],
                  ),
                );
                return;
              }
              final ownerID = robotProvider.users.firstWhere((element) => element.fullName == ownerController.text).id;
              final targetUserName = robotProvider.users.firstWhere((element) => element.fullName == recipientController.text).fullName;

              await APIService.postTask(
                taskID: 0.toString(),
                ownerID: ownerID,
                targetUser: targetUserName,
                moduleID: selectedModuleID!,
                drawerID: 0,
                xPose: mapController.position!.dx,
                yPose: mapController.position!.dy,
                yawPose: 0,
                robotName: "RB0",
                fleetName: "ROBAST",
              );
              Navigator.pop(context);
            },
          )
        ],
      ),
    );
  }

  Widget buildDetailsInputField({required String hintText, required TextEditingController controller}) {
    return Container(
      decoration: BoxDecoration(color: AppColors.grey, borderRadius: BorderRadius.circular(8)),
      padding: const EdgeInsets.all(20),
      child: TextFormField(
        decoration: InputDecoration.collapsed(hintText: hintText),
      ),
    );
  }
}
