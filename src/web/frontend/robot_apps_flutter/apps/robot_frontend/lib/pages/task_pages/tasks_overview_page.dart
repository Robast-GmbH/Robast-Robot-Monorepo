import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/pages/task_pages/tasks_history_page.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/expandable_subtask_tile.dart';

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
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  const Padding(
                    padding: EdgeInsets.only(left: 8),
                    child: Text(
                      'Aktiv',
                      style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
                    ),
                  ),
                  if (activeTask != null)
                    ExpandableSubtaskTile(
                      subtask: activeTask,
                    )
                  else
                    const Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(
                        'Kein Auftrag aktiv',
                        style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                      ),
                    ),
                  const Padding(
                    padding: EdgeInsets.only(left: 8, top: 16),
                    child: Text(
                      'In Warteschlange',
                      style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
                    ),
                  ),
                  if (queuedTasks.isEmpty)
                    const Padding(
                      padding: EdgeInsets.all(8.0),
                      child: Text(
                        'Keine Aufträge in Warteschlange',
                        style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                      ),
                    )
                  else
                    Expanded(
                      child: ListView(
                        children: List.generate(
                          queuedTasks.length,
                          (index) => Column(
                            children: [
                              ExpandableSubtaskTile(
                                subtask: queuedTasks[index],
                              ),
                              const SizedBox(height: 8),
                            ],
                          ),
                        ),
                      ),
                    ),
                  InkWell(
                    onTap: () {
                      Navigator.push(context, MaterialPageRoute<TasksHistoryPage>(builder: (context) => const TasksHistoryPage()));
                    },
                    child: const Padding(
                      padding: EdgeInsets.only(left: 8, top: 16),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.center,
                        children: [
                          Text(
                            'Vergangene Aufträge',
                            style: TextStyle(fontSize: 40, color: RobotColors.primaryText),
                          ),
                          SizedBox(
                            width: 8,
                          ),
                          Icon(
                            Icons.arrow_forward,
                            size: 40,
                            color: RobotColors.primaryIcon,
                          )
                        ],
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
