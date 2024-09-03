import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/pages/tasks_overview_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
import 'package:robot_frontend/widgets/dialogs/task_details_dialog.dart';

class MenuTasksOverview extends StatefulWidget {
  const MenuTasksOverview({super.key});

  @override
  State<MenuTasksOverview> createState() => _MenuTasksOverviewState();
}

class _MenuTasksOverviewState extends State<MenuTasksOverview> {
  late Future<void> fetchRobotTaskStatus;

  @override
  void initState() {
    super.initState();
    fetchRobotTaskStatus = Provider.of<TaskProvider>(context, listen: false).fetchRobotTaskStatus(robotName: 'rb_theron');
  }

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      header: InkWell(
        onTap: () {
          fetchRobotTaskStatus = Provider.of<TaskProvider>(context, listen: false).fetchRobotTaskStatus(robotName: 'rb_theron');
          setState(() {});
        },
        child: const Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Expanded(
              child: Text(
                'Auftragsübersicht',
                textAlign: TextAlign.start,
                style: TextStyle(
                  height: 0,
                  color: RobotColors.primaryText,
                  fontSize: 40,
                  fontWeight: FontWeight.w400,
                ),
              ),
            ),
            Icon(
              Icons.refresh,
              size: 48,
              color: RobotColors.primaryIcon,
            ),
          ],
        ),
      ),
      onPressed: () {},
      content: FutureBuilder(
          future: fetchRobotTaskStatus,
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return const Center(child: CircularProgressIndicator());
            }
            final taskProvider = Provider.of<TaskProvider>(context, listen: false);
            return Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const SizedBox(height: 16),
                Expanded(
                  child: RoundedButton(
                      onPressed: () {
                        if (taskProvider.activeTask == null) {
                          return;
                        }
                        showDialog(context: context, builder: (context) => TaskDetailsDialog(task: taskProvider.activeTask!));
                      },
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          const Text(
                            'Aktiv',
                            style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                          ),
                          Text(
                            taskProvider.activeTask != null
                                ? '${taskProvider.activeTask!.name} bei ${taskProvider.activeTask!.targetID}'
                                : 'Kein aktiver Auftrag',
                            style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                          ),
                        ],
                      )),
                ),
                const SizedBox(height: 16),
                Expanded(
                  child: RoundedButton(
                      onPressed: () {
                        if (taskProvider.queuedTasks.isEmpty) {
                          return;
                        }
                        showDialog(context: context, builder: (context) => TaskDetailsDialog(task: taskProvider.queuedTasks.firstOrNull!));
                      },
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          const Text(
                            'Nächster',
                            style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                          ),
                          Text(
                            taskProvider.queuedTasks.isNotEmpty
                                ? '${taskProvider.queuedTasks.first.name} bei ${taskProvider.queuedTasks.first.targetID}'
                                : 'Keine weiteren Aufträge',
                            style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                          ),
                        ],
                      )),
                ),
                const SizedBox(height: 16),
                Expanded(
                  child: RoundedButton(
                      onPressed: () {
                        Navigator.push(context, MaterialPageRoute(builder: (context) => const TasksOverviewPage()));
                      },
                      child: const Text(
                        'Mehr anzeigen',
                        style: TextStyle(fontSize: 32, color: RobotColors.secondaryText),
                      )),
                ),
              ],
            );
          }),
    );
  }
}
