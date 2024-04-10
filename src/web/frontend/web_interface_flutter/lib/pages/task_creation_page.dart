import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import 'package:provider/provider.dart';

import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/constants/gaps.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/details_input_field.dart';
import 'package:web_interface_flutter/widgets/optional_expanded.dart';
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
  final ownerController = TextEditingController(text: "Werner Tester");
  final recipientController = TextEditingController(text: "Werner Tester");
  final subjectController = TextEditingController();
  final mapController = MapController();
  int? selectedModuleID;

  Widget getTabWidget(Tabs tab) {
    switch (tab) {
      case Tabs.targetSelection:
        return buildTab(title: "1. Zielauswahl", child: buildTargetSelection(), tab: Tabs.targetSelection);

      case Tabs.moduleSelection:
        return buildTab(title: "2. Modulauswahl", child: buildModuleSelection(), tab: Tabs.moduleSelection);

      case Tabs.detailsInput:
        return buildTab(title: "3. Details", child: buildDetailsInput(), tab: Tabs.detailsInput);
    }
  }

  @override
  Widget build(BuildContext context) {
    return PageFrame(
      title: "Neuer Auftrag",
      color: AppColors.green,
      child: Padding(
        padding: Constants.mediumPadding,
        child: !kIsWeb
            ? Row(
                children: [
                  buildTab(title: "1. Zielauswahl", child: buildTargetSelection(), tab: Tabs.targetSelection),
                  Gaps.mediumHorizontal,
                  buildTab(title: "2. Modulauswahl", child: buildModuleSelection(), tab: Tabs.moduleSelection),
                  Gaps.mediumHorizontal,
                  buildTab(title: "3. Details", child: buildDetailsInput(), tab: Tabs.detailsInput),
                ],
              )
            : Stack(
                alignment: Alignment.bottomRight,
                children: [
                  getTabWidget(currentTab),
                  if (currentTab != Tabs.detailsInput) ...[
                    FloatingActionButton(
                      mini: true,
                      onPressed: () {
                        if (currentTab == Tabs.targetSelection) {
                          currentTab = Tabs.moduleSelection;
                        } else if (currentTab == Tabs.moduleSelection) {
                          currentTab = Tabs.detailsInput;
                        }
                        setState(() {});
                      },
                      backgroundColor: AppColors.green,
                      child: const Icon(Icons.arrow_forward_ios),
                    ),
                  ],
                ],
              ),
      ),
    );
  }

  Widget buildTab({required String title, required Widget child, required Tabs tab}) {
    return OptionalExpanded(
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
            padding: Constants.mediumPadding,
            decoration: BoxDecoration(
              color: AppColors.lightGrey,
              borderRadius: BorderRadius.circular(16),
            ),
            child: Stack(
              fit: currentTab == tab ? StackFit.expand : StackFit.loose,
              children: [
                currentTab == tab ? child : const SizedBox(),
                IgnorePointer(
                  child: Text(
                    title,
                    style: const TextStyle(fontSize: 32),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget buildTargetSelection() {
    return Padding(
      padding: const EdgeInsets.only(top: 48),
      child: RoboMap(controller: mapController),
    );
  }

  Widget buildModuleSelection() {
    return Padding(
      padding: Constants.largePadding,
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

  Future<void> showInsufficientInputDialog(Tabs tab, String content) async {
    await showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Angaben unkomplett"),
        content: Text(content),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.pop(context);
              currentTab = tab;
              setState(() {});
            },
            child: const Text("Okay"),
          )
        ],
      ),
    );
  }

  Widget buildDetailsInput() {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Padding(
      padding: Constants.mediumPadding,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.end,
        children: [
          Gaps.largeVertical,
          DetailsInputField(
            hintText: "Auftraggeber",
            controller: ownerController,
            autocompleteValues: robotProvider.users.map((e) => e.fullName).toList(),
          ),
          Gaps.mediumVertical,
          DetailsInputField(
            hintText: "Empfänger",
            controller: recipientController,
            autocompleteValues: robotProvider.users.map((e) => e.fullName).toList(),
          ),
          Gaps.mediumVertical,
          Expanded(
            child: DetailsInputField(
              hintText: "Betreff",
              controller: subjectController,
            ),
          ),
          Gaps.mediumVertical,
          RoundedButton(
            text: "Auftrag bestätigen",
            color: AppColors.green,
            borderRadius: BorderRadius.circular(8),
            padding: EdgeInsets.zero,
            onTap: () async {
              if (mapController.points.isEmpty) {
                await showInsufficientInputDialog(Tabs.targetSelection, "Es wurde noch kein Ziel ausgewählt");
                return;
              }
              if (selectedModuleID == null) {
                await showInsufficientInputDialog(Tabs.moduleSelection, "Es wurde noch kein Schubladen-Modul ausgewählt");
                return;
              }
              if (ownerController.text.isEmpty || recipientController.text.isEmpty) {
                await showInsufficientInputDialog(Tabs.detailsInput, "Bitte alle Felder ausfüllen, um Aufgabe zu erstellen.");
                return;
              }
              try {
                final ownerID = robotProvider.users.firstWhere((element) => element.fullName == ownerController.text).id;
                final targetUserName = robotProvider.users.firstWhere((element) => element.fullName == recipientController.text).fullName;
                final navPoint = mapController.positionsAsNavPoints().first;
                Navigator.pop(context);
                // TODO: Implement API call to create task
              } catch (e) {
                await showDialog(
                  context: context,
                  builder: (context) => AlertDialog(
                    title: const Text("Angaben fehlerhaft"),
                    content: const Text("Sender und/oder Empfänger exisitieren nicht."),
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
              }
            },
          )
        ],
      ),
    );
  }
}
