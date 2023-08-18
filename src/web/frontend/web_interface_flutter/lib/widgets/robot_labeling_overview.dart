import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
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
    return Padding(
      padding: const EdgeInsets.all(32),
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
                              await APIService.relabelDrawer(moduleID, 0, controller.text, "RB0");
                              await robotProvider.updateModules();
                            }
                            Navigator.pop(context);
                            setState(() {});
                          },
                          child: const Text("Übernehmen")),
                      TextButton(
                          onPressed: () {
                            Navigator.pop(context);
                          },
                          child: const Text("Abbrechen"))
                    ],
                  ));
        },
      ),
    );
  }
}
