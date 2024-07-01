import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/task_provider.dart';

class RobotTaskView extends StatefulWidget {
  const RobotTaskView({required this.robotName, super.key});

  final String robotName;

  @override
  State<RobotTaskView> createState() => _RobotTaskViewState();
}

class _RobotTaskViewState extends State<RobotTaskView> {
  @override
  void initState() {
    super.initState();
    Provider.of<RMFProvider>(context, listen: false).startPeriodicTasksUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RMFProvider>(context, listen: false).stopPeriodicTasksUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Selector<RMFProvider, List<Task>>(
      selector: (_, provider) => provider.tasks,
      builder: (context, tasks, child) {
        final filteredTasks = tasks.where((task) => task.assigneeName == widget.robotName).toList();
        return ListView.builder(
          itemCount: filteredTasks.length,
          itemBuilder: (context, index) {
            return Card(
              elevation: 5,
              margin: const EdgeInsets.all(16),
              child: ListTile(
                leading: CircleAvatar(
                  child: Text(filteredTasks[index].id.characters.last),
                ),
                title: Text('Task ID: ${filteredTasks[index].id}'),
                subtitle: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Category: ${filteredTasks[index].taskType}'),
                    const Text('State: To be implemented'),
                  ],
                ),
                trailing: const Icon(Icons.more_vert),
              ),
            );
          },
        );
      },
    );
  }
}
