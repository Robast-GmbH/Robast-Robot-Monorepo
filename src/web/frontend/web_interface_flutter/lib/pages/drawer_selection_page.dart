import 'package:flutter/material.dart';
import 'package:web_interface_flutter/pages/task_creation_page.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class DrawerSelectionPage extends StatefulWidget {
  const DrawerSelectionPage({super.key, required this.targetPosition});
  final Offset targetPosition;
  @override
  State<DrawerSelectionPage> createState() => _DrawerSelectionPageState();
}

class _DrawerSelectionPageState extends State<DrawerSelectionPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: const Text("Wähle eine Schublade"),
      ),
      body: Center(child: RobotClone(
        onPressed: (index) {
          Navigator.push(
            context,
            MaterialPageRoute(
              builder: (context) => TaskCreationPage(
                targetPosition: widget.targetPosition,
                moduleID: index,
                drawerID: 0,
              ),
            ),
          );
        },
      )),
    );
  }
}
