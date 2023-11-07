import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';

class TasksOverview extends StatefulWidget {
  const TasksOverview({super.key});

  @override
  State<TasksOverview> createState() => _TasksOverviewState();
}

class _TasksOverviewState extends State<TasksOverview> {
  @override
  Widget build(BuildContext context) {
    final tasks = Provider.of<RobotProvider>(context).tasks;
    return Padding(
      padding: const EdgeInsets.symmetric(
        horizontal: 8,
        vertical: 2,
      ),
      child: ListView(
        children: List.generate(
          tasks.length,
          (index) => Card(
              child: Row(
            children: [
              const SizedBox(width: 8),
              Expanded(
                child: Padding(
                  padding: Constants.smallPadding,
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text("ID: ${tasks[index].id}"),
                      Text("Robot: ${tasks[index].robotName}"),
                      Text("Owner ID: ${tasks[index].fleetName}"),
                      //...tasks[index].actions.map((e) => Text(e.runtimeType.toString())).toList()
                      // Text("Target ID: ${tasks[index].targetID}"),
                      // Text("Target x: ${tasks[index].xPose}"),
                      // Text("Target y: ${tasks[index].yPose}"),
                      // Text("Target yaw: ${tasks[index].yawPose}"),
                      // Text("Module ID: ${tasks[index].moduleID}"),
                      // Text("Drawer ID: ${tasks[index].drawerID}"),
                      // Text("Finished: ${tasks[index].finished}"),
                    ],
                  ),
                ),
              ),
              IconButton(onPressed: () {}, icon: const Icon(Icons.more_vert))
            ],
          )),
        ),
      ),
    );
  }
}
