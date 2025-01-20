import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/widgets/task_tile.dart';

class TasksHistoryPage extends StatefulWidget {
  const TasksHistoryPage({required this.robotName, super.key});

  final String robotName;

  @override
  State<TasksHistoryPage> createState() => _TasksHistoryPageState();
}

class _TasksHistoryPageState extends State<TasksHistoryPage> {
  late Future<List<Task>?> fetchTasks;
  int pageIndex = 0;

  @override
  void initState() {
    super.initState();
    fetchTasks = Provider.of<TaskProvider>(context, listen: false).fetchTasks(robotName: widget.robotName, limit: 10, offset: 0);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Vergangene Aufträge'),
      ),
      body: FutureBuilder(
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
                  ElevatedButton(
                    onPressed: () {
                      setState(
                        () {
                          fetchTasks =
                              Provider.of<TaskProvider>(context, listen: false).fetchTasks(robotName: widget.robotName, limit: 10, offset: pageIndex * 10);
                        },
                      );
                    },
                    child: const Text('Erneut versuchen'),
                  ),
                ],
              ),
            );
          }
          return Padding(
            padding: const EdgeInsets.all(8),
            child: ListView.builder(
              itemCount: tasks.length + 1,
              itemBuilder: (context, index) {
                if (index < tasks.length) {
                  return TaskTile(task: tasks[index]);
                }
                return Padding(
                  padding: const EdgeInsets.all(8),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      if (pageIndex > 0)
                        ElevatedButton(
                          onPressed: () {
                            setState(
                              () {
                                pageIndex--;
                                fetchTasks = Provider.of<TaskProvider>(context, listen: false)
                                    .fetchTasks(robotName: widget.robotName, limit: 10, offset: pageIndex * 10);
                              },
                            );
                          },
                          child: const Text('Zurück'),
                        ),
                      if (tasks.length == 10)
                        ElevatedButton(
                          onPressed: () {
                            setState(
                              () {
                                pageIndex++;
                                fetchTasks = Provider.of<TaskProvider>(context, listen: false)
                                    .fetchTasks(robotName: widget.robotName, limit: 10, offset: pageIndex * 10);
                              },
                            );
                          },
                          child: const Text('Weiter'),
                        ),
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
