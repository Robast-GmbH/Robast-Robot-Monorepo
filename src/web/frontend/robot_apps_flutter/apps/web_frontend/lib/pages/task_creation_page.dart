import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/widgets/delivery_task_creation_view.dart';
import 'package:web_frontend/widgets/multi_drop_off_task_creation_view.dart';
import 'package:web_frontend/widgets/patrol_task_creation_view.dart';

class TaskCreationPage extends StatefulWidget {
  const TaskCreationPage({super.key});

  @override
  State<TaskCreationPage> createState() => _TaskCreationPageState();
}

class _TaskCreationPageState extends State<TaskCreationPage> {
  String? dropdownValue = 'Patrol';
  final _taskCreationController = TaskCreationController();

  Widget buildTaskCreationWidget() {
    if (dropdownValue == 'Patrol') {
      return PatrolTaskCreationView(
        controller: _taskCreationController,
      );
    } else if (dropdownValue == 'Delivery') {
      return DeliveryTaskCreationView(
        controller: _taskCreationController,
      );
    } else if (dropdownValue == 'Multi Dropoff') {
      return MultiDropOffTaskCreationView(
        controller: _taskCreationController,
      );
    } else {
      return const Placeholder();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Task Creation'),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: dropdownValue == null
            ? null
            : () async {
                if (_taskCreationController.validateTask(type: dropdownValue!)) {
                  await showDialog<AlertDialog>(
                    context: context,
                    builder: (context) {
                      return AlertDialog(
                        title: const Text('Nice!'),
                        content: Text('${dropdownValue ?? ''} task has been created.'),
                        actions: [
                          TextButton(
                            onPressed: () async {
                              await Provider.of<RMFProvider>(context, listen: false).dispatchTask(
                                taskType: dropdownValue!,
                                controller: _taskCreationController,
                              );
                              if (context.mounted) {
                                Navigator.pop(context);
                              }
                            },
                            child: const Text('Dispatch'),
                          ),
                          TextButton(
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Cancel'),
                          ),
                        ],
                      );
                    },
                  );

                  if (context.mounted) {
                    Navigator.pop(context);
                  }
                } else {
                  await showDialog<AlertDialog>(
                    context: context,
                    builder: (context) {
                      return AlertDialog(
                        title: const Text('Invalid Task'),
                        content: const Text('Please check task data.'),
                        actions: [
                          TextButton(
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('OK'),
                          ),
                        ],
                      );
                    },
                  );
                }
              },
        child: const Icon(Icons.send),
      ),
      body: Column(
        children: [
          const SizedBox(height: 8),
          Row(
            children: [
              const Padding(
                padding: EdgeInsets.symmetric(horizontal: 16),
                child: Text('Task Type:'),
              ),
              Expanded(
                child: DropdownButton<String>(
                  padding: const EdgeInsets.only(right: 16),
                  isExpanded: true,
                  value: dropdownValue,
                  hint: const Text('Select Task Type'),
                  onChanged: (String? newValue) {
                    setState(() {
                      dropdownValue = newValue;
                    });
                  },
                  items: <String>['Patrol', 'Delivery', 'Multi Dropoff'].map<DropdownMenuItem<String>>((String value) {
                    return DropdownMenuItem<String>(
                      value: value,
                      child: Text(value),
                    );
                  }).toList(),
                ),
              ),
            ],
          ),
          const Divider(color: Colors.grey),
          Expanded(child: buildTaskCreationWidget()),
        ],
      ),
    );
  }
}
