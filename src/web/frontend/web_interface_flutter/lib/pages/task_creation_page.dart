import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class TaskCreationPage extends StatefulWidget {
  const TaskCreationPage({
    super.key,
    required this.targetPosition,
    required this.moduleID,
    required this.drawerID,
  });

  final Offset targetPosition;
  final int moduleID;
  final int drawerID;
  @override
  State<TaskCreationPage> createState() => _TaskCreationPageState();
}

class _TaskCreationPageState extends State<TaskCreationPage> {
  String? sender;
  String? receiver;

  @override
  void initState() {
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
      body: Center(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 480),
            child: Column(
              children: [
                Row(
                  mainAxisSize: MainAxisSize.max,
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    const SizedBox(
                      height: 16,
                    ),
                    const Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text("Sender:"),
                        SizedBox(
                          height: 64,
                        ),
                        Text("Empfänger:"),
                      ],
                    ),
                    const SizedBox(
                      width: 64,
                    ),
                    Expanded(
                      child: Column(
                        children: [
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
                          const SizedBox(
                            height: 32,
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
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
      floatingActionButton: FloatingActionButton(
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
          final targetUserName = robotProvider.users.firstWhere((element) => element.fullName == receiver).fullName;

          APIService.postTask(
            taskID: 0.toString(),
            ownerID: ownerID,
            targetUser: targetUserName,
            moduleID: widget.moduleID,
            drawerID: widget.drawerID,
            xPose: widget.targetPosition.dx,
            yPose: widget.targetPosition.dy,
            yawPose: 0,
            robotName: "RB0",
            fleetName: "ROBAST",
          );
          Navigator.pop(context);
          Navigator.pop(context);
        },
        child: const Icon(Icons.start),
      ),
    );
  }
}
