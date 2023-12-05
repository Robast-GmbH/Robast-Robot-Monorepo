import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class RobotLabelingOverview extends StatefulWidget {
  const RobotLabelingOverview({super.key});

  @override
  State<RobotLabelingOverview> createState() => _RobotLabelingOverviewState();
}

class _RobotLabelingOverviewState extends State<RobotLabelingOverview> {
  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Stack(
      children: [
        Padding(
          padding: Constants.largePadding,
          child: RobotClone(
            onPressed: (moduleID) async {
              final controller = TextEditingController();
              await showDialog(
                context: context,
                builder: (context) => AlertDialog(
                  title: const Text("Label ändern"),
                  content: TextField(
                    controller: controller,
                  ),
                  actions: [
                    TextButton(
                      onPressed: () async {
                        if (controller.text.isNotEmpty) {
                          await APIService.relabelDrawer(
                            moduleID: moduleID,
                            drawerID: 0,
                            label: controller.text,
                            robotName: "RB0",
                          );
                          await robotProvider.updateModules();
                        }
                        Navigator.pop(context);
                        setState(() {});
                      },
                      child: const Text("Übernehmen"),
                    ),
                    TextButton(
                      onPressed: () {
                        Navigator.pop(context);
                      },
                      child: const Text("Abbrechen"),
                    ),
                  ],
                ),
              );
            },
          ),
        ),
        Align(
          alignment: Alignment.bottomRight,
          child: Padding(
            padding: Constants.mediumPadding,
            child: ElevatedButton(
              style: const ButtonStyle(
                minimumSize: MaterialStatePropertyAll(
                  Size(256, 64),
                ),
              ),
              child: const Text("Module zurücksetzen"),
              onPressed: () async {
                await APIService.resetModules(robotName: "RB0");
              },
            ),
          ),
        ),
      ],
    );
  }
}
