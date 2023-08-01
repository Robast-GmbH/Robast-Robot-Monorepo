import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/widgets/drawer_tile.dart';

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
      body: robotProvider.robots.isEmpty
          ? const Center(
              child: Text("Aktuell sind keine Roboter verfügbar."),
            )
          : Row(
              children: [
                Expanded(
                  child: Container(
                    decoration: const BoxDecoration(color: Colors.white, boxShadow: [
                      BoxShadow(
                        color: Colors.grey, // Shadow color
                        offset: Offset(0, 2), // Changes position of shadow
                        blurRadius: 2, // Changes size of shadow
                        spreadRadius: 1, // Expands the shadow
                      ),
                    ]),
                    child: ListView(
                        children: ListTile.divideTiles(
                      context: context,
                      tiles: List.generate(
                        robotProvider.robots.length,
                        (index) => ListTile(
                          selected: selectedRobotIndex == index,
                          title: Text(robotProvider.robots[index].name),
                          onTap: () {
                            selectedRobotIndex = index;
                            setState(() {});
                          },
                        ),
                      ),
                    ).toList()),
                  ),
                ),
                Expanded(
                  flex: 5,
                  child: Center(
                    child: Padding(
                      padding: const EdgeInsets.all(16),
                      child: SingleChildScrollView(
                        child: Column(
                          children: robotProvider.modules[robots[selectedRobotIndex]]
                                  ?.map((e) => DrawerTile(
                                        moduleID: e.moduleID,
                                        drawerID: e.drawerID,
                                        robotName: robots[selectedRobotIndex].name,
                                      ))
                                  .toList() ??
                              [],
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
    );
  }
}
