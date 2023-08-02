import 'package:flutter/material.dart';
import 'package:web_interface_flutter/widgets/docker_overview.dart';
import 'package:web_interface_flutter/widgets/rosbag_overview.dart';
import 'package:web_interface_flutter/widgets/status_overview.dart';
import 'package:web_interface_flutter/widgets/tasks_overview.dart';

class AdminPage extends StatefulWidget {
  const AdminPage({super.key});

  @override
  State<AdminPage> createState() => _AdminPageState();
}

class _AdminPageState extends State<AdminPage> {
  final menuPoints = ["docker", "rosbag", "status", "log", "tasks"];
  final menuIcons = [Icons.settings, Icons.backpack, Icons.battery_5_bar, Icons.file_open, Icons.task];
  final menuWidget = [
    const DockerOverview(),
    const RosbagOverview(),
    const StatusOverview(),
    const SizedBox(),
    const TasksOverview(),
  ];
  int selectedIndex = 0;
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: const Text("Admin"),
      ),
      body: Row(
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
                  menuPoints.length,
                  (index) => ListTile(
                    leading: Icon(menuIcons[index]),
                    selected: selectedIndex == index,
                    title: Text(menuPoints[index]),
                    onTap: () {
                      selectedIndex = index;
                      setState(() {});
                    },
                  ),
                ),
              ).toList()),
            ),
          ),
          Expanded(
            flex: 5,
            child: Center(child: menuWidget[selectedIndex]),
          ),
        ],
      ),
    );
  }
}
