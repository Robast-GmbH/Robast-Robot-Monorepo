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

  late Timer refreshTimer;

  void setTimer() {
    refreshTimer = Timer.periodic(const Duration(seconds: 5), (timer) {
      print("Update Robots");
      Provider.of<RobotProvider>(context, listen: false).updateRobots();
      setState(() {});
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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          backgroundColor: Theme.of(context).colorScheme.inversePrimary,
          title: const Text("Manuelle Kontrolle"),
        ),
        body: Center(child: RobotClone(
          onPressed: (moduleID) async {
            final isClosed = Random().nextBool();
            if (isClosed) {
              APIService.openDrawer("Robo", moduleID, 0);
            } else {
              APIService.closeDrawer("Robo", moduleID, 0);
            }
          },
        )));
  }
}
