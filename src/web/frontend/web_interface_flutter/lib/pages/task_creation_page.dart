import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class TaskCreationPage extends StatefulWidget {
  const TaskCreationPage({super.key, required this.targetPosition});

  final Offset targetPosition;

  @override
  State<TaskCreationPage> createState() => _TaskCreationPageState();
}

class _TaskCreationPageState extends State<TaskCreationPage> {
  final targetXController = TextEditingController();
  final targetYController = TextEditingController();
  final moduleIDController = TextEditingController();
  final drawerIDController = TextEditingController();
  String? sender;
  String? receiver;

  @override
  void initState() {
    targetXController.text = widget.targetPosition.dx.toStringAsFixed(2);
    targetYController.text = widget.targetPosition.dy.toStringAsFixed(2);
    moduleIDController.text = "0";
    drawerIDController.text = "0";
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: const Text("Lieferauftrag erstellen"),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 320),
          child: SingleChildScrollView(
            child: Column(
              children: [
                Row(
                  children: [
                    const Text("Ziel X:"),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: TextFormField(
                        keyboardType: TextInputType.number,
                        controller: targetXController,
                      ),
                    ),
                    const SizedBox(
                      width: 32,
                    ),
                    const Text("Ziel Y:"),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: TextFormField(
                        keyboardType: TextInputType.number,
                        controller: targetYController,
                      ),
                    ),
                  ],
                ),
                const SizedBox(
                  height: 16,
                ),
                Row(
                  children: [
                    const Text("Sender:"),
                    const SizedBox(
                      width: 16,
                    ),
                    DropdownMenu<String>(
                      onSelected: (value) {
                        sender = value;
                      },
                      dropdownMenuEntries: robotProvider.users
                          .map((e) => e.fullName)
                          .map(
                            (e) => DropdownMenuEntry(value: e, label: e),
                          )
                          .toList(),
                    ),
                  ],
                ),
                const SizedBox(
                  height: 8,
                ),
                Row(
                  children: [
                    const Text("Empfänger:"),
                    const SizedBox(
                      width: 16,
                    ),
                    DropdownMenu<String>(
                      onSelected: (value) {
                        receiver = value;
                      },
                      dropdownMenuEntries: robotProvider.users
                          .map((e) => e.fullName)
                          .map(
                            (e) => DropdownMenuEntry(value: e, label: e),
                          )
                          .toList(),
                    ),
                  ],
                ),
                Row(
                  children: [
                    const Text("Modul ID:"),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: TextFormField(
                        keyboardType: TextInputType.number,
                        controller: moduleIDController,
                      ),
                    ),
                    const SizedBox(
                      width: 32,
                    ),
                    const Text("Schublade:"),
                    const SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: TextFormField(
                        keyboardType: TextInputType.number,
                        controller: drawerIDController,
                      ),
                    ),
                  ],
                ),
                const SizedBox(
                  height: 32,
                ),
                ElevatedButton.icon(
                  onPressed: () {
                    if (sender == null || receiver == null) {
                      showDialog(
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
                    final ownerID = robotProvider.users.firstWhere((element) => element.fullName == sender).id;
                    final targetID = robotProvider.users.firstWhere((element) => element.fullName == receiver).id;
                    final moduleID = int.tryParse(moduleIDController.text);
                    final drawerID = int.tryParse(drawerIDController.text);
                    final xPose = double.tryParse(targetXController.text);
                    final yPose = double.tryParse(targetYController.text);
                    final isValidTask = moduleID != null && drawerID != null && xPose != null && yPose != null;
                    if (isValidTask) {
                      APIService.postTask(ownerID: ownerID, targetID: targetID, moduleID: moduleID, drawerID: drawerID, xPose: xPose, yPose: yPose, yawPose: 0);
                    } else {
                      showDialog(
                        context: context,
                        builder: (context) => AlertDialog(
                          title: const Text("Angaben inkorrekt"),
                          content: const Text("Bitte Felder überprüfen und ggf. korrigieren, um Aufgabe zu erstellen."),
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
                  icon: const Icon(Icons.add_task_outlined),
                  label: const Text(
                    "Auftrag erstellen",
                  ),
                )
              ],
            ),
          ),
        ),
      ),
    );
  }
}
