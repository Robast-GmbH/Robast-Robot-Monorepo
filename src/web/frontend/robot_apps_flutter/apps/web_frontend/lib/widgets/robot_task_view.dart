import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/pages/tasks_history_page.dart';
import 'package:web_frontend/widgets/expandable_subtask_tile.dart';

class RobotTaskView extends StatefulWidget {
  const RobotTaskView({required this.robotName, super.key});

  final String robotName;

  @override
  State<RobotTaskView> createState() => _RobotTaskViewState();
}

class _RobotTaskViewState extends State<RobotTaskView> {
  late Future<RobotTaskStatus?> fetchRobotTasks;
  @override
  void initState() {
    super.initState();
    fetchRobotTasks = Provider.of<TaskProvider>(context, listen: false).getRobotTasks(robotName: widget.robotName);
  }

  @override
  void deactivate() {
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: FutureBuilder<RobotTaskStatus?>(
        future: fetchRobotTasks,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const CircularProgressIndicator();
          }
          if (snapshot.data == null) {
            return const Text('Fail not handled');
          }
          final robotTaskStatus = snapshot.data!;
          final activeTask = robotTaskStatus.activeTask;
          final queuedTasks = robotTaskStatus.queuedTasks;
          return Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                const Padding(
                  padding: EdgeInsets.only(left: 8, top: 16),
                  child: Text(
                    'Aktiv',
                    style: TextStyle(fontSize: 24, color: WebColors.primaryText),
                  ),
                ),
                if (activeTask != null)
                  ExpandableSubtaskTile(
                    subtask: activeTask,
                  )
                else
                  const Padding(
                    padding: EdgeInsets.all(8),
                    child: Text(
                      'Kein Auftrag aktiv',
                      style: TextStyle(color: WebColors.secondaryText, fontSize: 20),
                    ),
                  ),
                const Padding(
                  padding: EdgeInsets.only(left: 8, top: 16),
                  child: Text(
                    'In Warteschlange',
                    style: TextStyle(fontSize: 24, color: WebColors.primaryText),
                  ),
                ),
                if (queuedTasks.isEmpty)
                  const Padding(
                    padding: EdgeInsets.all(8),
                    child: Text(
                      'Keine Aufträge in Warteschlange',
                      style: TextStyle(color: WebColors.secondaryText, fontSize: 20),
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
                    Navigator.push(
                      context,
                      MaterialPageRoute<TasksHistoryPage>(
                        builder: (context) => TasksHistoryPage(
                          robotName: widget.robotName,
                        ),
                      ),
                    );
                  },
                  child: const Padding(
                    padding: EdgeInsets.only(left: 8, top: 16, bottom: 16),
                    child: Row(
                      children: [
                        Text(
                          'Vergangene Aufträge',
                          style: TextStyle(fontSize: 24, color: WebColors.primaryText),
                        ),
                        SizedBox(
                          width: 8,
                        ),
                        Icon(
                          Icons.arrow_forward,
                          size: 24,
                          color: WebColors.primaryIcon,
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}
