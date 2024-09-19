import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/task_tile.dart';
import 'package:shared_data_models/shared_data_models.dart';

class TasksHistoryPage extends StatefulWidget {
  const TasksHistoryPage({super.key});

  @override
  State<TasksHistoryPage> createState() => _TasksHistoryPageState();
}

class _TasksHistoryPageState extends State<TasksHistoryPage> {
  late Future<List<Task>?> fetchTasks;
  int pageIndex = 0;

  @override
  initState() {
    super.initState();
    fetchTasks = Provider.of<TaskProvider>(context, listen: false).fetchTasks(limit: 10, offset: 0);
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Vergangene Aufträge',
      child: FutureBuilder(
        future: fetchTasks,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(child: CircularProgressIndicator());
          }
          final tasks = snapshot.data;
          if (tasks == null) {
            return Center(
              child: Column(
                children: [
                  const Text('Fehler beim Laden der Aufträge'),
                  CustomElevatedButton(
                      onPressed: () {
                        setState(
                          () {
                            fetchTasks = Provider.of<TaskProvider>(context, listen: false).fetchTasks(limit: 10, offset: pageIndex * 10);
                          },
                        );
                      },
                      label: 'Erneut versuchen')
                ],
              ),
            );
          }
          return Padding(
            padding: const EdgeInsets.all(64),
            child: ListView.builder(
              itemCount: tasks.length + 1,
              itemBuilder: (context, index) {
                if (index < tasks.length) {
                  return TaskTile(task: tasks[index]);
                }
                return Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      if (pageIndex > 0)
                        CustomElevatedButton(
                            onPressed: () {
                              setState(
                                () {
                                  pageIndex--;
                                  fetchTasks = Provider.of<TaskProvider>(context, listen: false).fetchTasks(limit: 10, offset: pageIndex * 10);
                                },
                              );
                            },
                            label: 'Zurück'),
                      if (tasks.length == 10)
                        CustomElevatedButton(
                            onPressed: () {
                              setState(
                                () {
                                  pageIndex++;
                                  fetchTasks = Provider.of<TaskProvider>(context, listen: false).fetchTasks(limit: 10, offset: pageIndex * 10);
                                },
                              );
                            },
                            label: 'Weiter')
                    ],
                  ),
                );
              },
            ),
          );
        },
      ),
    );
  }
}
