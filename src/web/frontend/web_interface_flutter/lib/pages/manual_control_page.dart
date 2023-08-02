import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/widgets/drawer_tile.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class ManualControlPage extends StatefulWidget {
  const ManualControlPage({super.key});

  @override
  State<ManualControlPage> createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  int selectedRobotIndex = 0;
  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    final robots = robotProvider.robots;
    return Scaffold(
        appBar: AppBar(
          backgroundColor: Theme.of(context).colorScheme.inversePrimary,
          title: const Text("Manuelle Kontrolle"),
        ),
        body: Center(child: const RobotClone()));
  }
}
