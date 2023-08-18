import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class ManualControlPage extends StatefulWidget {
  const ManualControlPage({super.key});

  @override
  State<ManualControlPage> createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  int selectedRobotIndex = 0;
  final openColor = const Color.fromRGBO(128, 128, 128, 184 / 255);
  final closedColor = const Color.fromRGBO(0, 155, 155, 25 / 255);
  late Timer refreshTimer;

  void setTimer() {
    refreshTimer = Timer.periodic(const Duration(seconds: 5), (timer) async {
      debugPrint("Update Modules");
      await Provider.of<RobotProvider>(context, listen: false).updateModules();
    });
  }

  @override
  void initState() {
    super.initState();
    setTimer();
  }

  @override
  void dispose() {
    refreshTimer.cancel();
    super.dispose();
  }

  Future<void> showChangeDrawerStatusDialog(bool isOpening) async {
    await showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            title: Text("Schublade wird ${isOpening ? "geöffnet" : "geschlossen"}."),
            actions: [
              TextButton(
                  onPressed: () {
                    Navigator.pop(context);
                  },
                  child: const Text("Ok"))
            ],
          );
        });
  }

  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return PageFrame(
      color: AppColors.blue,
      title: "Freie Schubladenauswahl",
      child: Container(
        margin: const EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: AppColors.lightGrey,
          borderRadius: BorderRadius.circular(16),
        ),
        child: Stack(
          children: [
            Center(
              child: Padding(
                padding: const EdgeInsets.all(32),
                child: RobotClone(
                  displayStatus: true,
                  onPressed: (moduleID) async {
                    try {
                      final module = robotProvider.modules["RB0"]?.firstWhere((element) => element.moduleID == moduleID);

                      if (module!.status == "Opened" && module.type == ModuleType.Electrical) {
                        await APIService.closeDrawer("RB0", moduleID, module.drawerID);
                        await showChangeDrawerStatusDialog(false);
                      } else if (module.status == "Closed") {
                        await APIService.openDrawer("RB0", moduleID, module.drawerID);
                        await showChangeDrawerStatusDialog(true);
                      } else {
                        return;
                      }
                      await robotProvider.updateModules();
                    } catch (e) {
                      debugPrint(e.toString());
                    }
                  },
                ),
              ),
            ),
            const Padding(
              padding: EdgeInsets.all(16),
              child: Text(
                "1. Modul wählen",
                style: TextStyle(fontSize: 18),
              ),
            )
          ],
        ),
      ),
    );
  }
}
