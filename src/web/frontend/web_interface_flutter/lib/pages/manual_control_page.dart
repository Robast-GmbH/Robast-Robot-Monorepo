import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/main.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
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
      print("Update Modules");
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
                  child: Text("Ok"))
            ],
          );
        });
  }

  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return Scaffold(
        appBar: AppBar(
          backgroundColor: Theme.of(context).colorScheme.inversePrimary,
          title: const Text("Manuelle Kontrolle"),
        ),
        body: Center(
            child: RobotClone(
          displayStatus: true,
          onPressed: (moduleID) async {
            try {
              final module = robotProvider.modules["RB0"]?.firstWhere((element) => element.moduleID == moduleID);

              if (module!.status == "open") {
                await APIService.closeDrawer("RB0", moduleID, module.drawerID);
                await showChangeDrawerStatusDialog(false);
              } else {
                await APIService.openDrawer("RB0", moduleID, module.drawerID);
                await showChangeDrawerStatusDialog(true);
              }
              await robotProvider.updateModules();
            } catch (e) {
              debugPrint(e.toString());
            }
          },
        )));
  }
}
