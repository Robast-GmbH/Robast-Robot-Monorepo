import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/drawer_module.dart';
import 'package:web_interface_flutter/pages/task_creation_page.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class DrawerSelectionPage extends StatefulWidget {
  const DrawerSelectionPage({super.key, required this.targetPosition});
  final Offset targetPosition;
  @override
  State<DrawerSelectionPage> createState() => _DrawerSelectionPageState();
}

class _DrawerSelectionPageState extends State<DrawerSelectionPage> {
  int? selectedDrawerIndex;
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text("Wähle eine Schublade"),
      ),
      body: Center(
          child: RobotClone(
        selectedDrawerID: selectedDrawerIndex,
        onPressed: (index) {
          if (index != selectedDrawerIndex) {
            selectedDrawerIndex = index;
          } else {
            selectedDrawerIndex = null;
          }
          setState(() {});
        },
      )),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          if (selectedDrawerIndex != null) {
            Navigator.push(
              context,
              MaterialPageRoute(
                builder: (context) => TaskCreationPage(
                  targetPosition: widget.targetPosition,
                  drawer: DrawerModule(
                    moduleID: selectedDrawerIndex!,
                    drawerID: 0,
                  ),
                ),
              ),
            );
          }
        },
        child: Icon(Icons.check),
      ),
    );
  }
}
