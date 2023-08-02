import 'package:flutter/material.dart';
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
    return Scaffold(
        appBar: AppBar(
          backgroundColor: Theme.of(context).colorScheme.inversePrimary,
          title: const Text("Manuelle Kontrolle"),
        ),
        body: const Center(child: RobotClone()));
  }
}
