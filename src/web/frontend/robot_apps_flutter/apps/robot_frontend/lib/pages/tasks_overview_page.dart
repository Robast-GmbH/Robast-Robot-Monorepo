import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/expandable_task_tile.dart';

class TasksOverviewPage extends StatefulWidget {
  const TasksOverviewPage({super.key});

  @override
  State<TasksOverviewPage> createState() => _TasksOverviewPageState();
}

class _TasksOverviewPageState extends State<TasksOverviewPage> {
  late Future<void> fetchRobotTasks;

  @override
  void initState() {
    super.initState();
    fetchRobotTasks = Provider.of<TaskProvider>(context, listen: false).fetchRobotTaskStatus(
      robotName: 'rb_theron',
    );
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Auftragsübersicht',
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 128, vertical: 64),
        child: Center(
          child: FutureBuilder<void>(
            future: fetchRobotTasks,
            builder: (context, snapshot) {
              if (snapshot.connectionState != ConnectionState.done) {
                return const CircularProgressIndicator();
              }
              final activeTask = Provider.of<TaskProvider>(context, listen: false).activeTask;
              final queuedTasks = Provider.of<TaskProvider>(context, listen: false).queuedTasks;
              return Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Padding(
                    padding: EdgeInsets.only(left: 8),
                    child: Text(
                      'Aktiv',
                      style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
                    ),
                  ),
                  if (activeTask != null)
                    ExpandableTaskTile(
                      task: activeTask,
                    ),
                  const Padding(
                    padding: EdgeInsets.only(left: 8, top: 16),
                    child: Text(
                      'In Warteschlange',
                      style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
                    ),
                  ),
                  Expanded(
                    child: ListView(
                      children: List.generate(
                        queuedTasks.length,
                        (index) => ExpandableTaskTile(
                          task: queuedTasks[index],
                        ),
                      ),
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
  }
}
