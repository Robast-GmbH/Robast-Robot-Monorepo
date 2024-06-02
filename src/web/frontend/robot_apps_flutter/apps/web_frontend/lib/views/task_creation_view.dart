import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';
import 'package:web_frontend/widgets/delivery_task_creation_view.dart';
import 'package:web_frontend/widgets/patrol_task_creation_view.dart';

class TaskCreationView extends StatefulWidget {
  const TaskCreationView({super.key});

  @override
  State<TaskCreationView> createState() => _TaskCreationViewState();
}

class _TaskCreationViewState extends State<TaskCreationView> {
  String? dropdownValue = 'Patrol';
  final _taskCreationController = TaskCreationController();

  Future<void> dispatchTask() async {
    if (dropdownValue == 'Patrol') {
      await Provider.of<RMFProvider>(context, listen: false).dispatchPatrolTask(
        places: _taskCreationController.places,
        rounds: _taskCreationController.rounds!,
      );
    } else {
      await Provider.of<RMFProvider>(context, listen: false).dispatchDeliveryTask(
        pickup: _taskCreationController.pickupNode!,
        dropoff: _taskCreationController.dropoffNode!,
        drawerID: _taskCreationController.drawerID!,
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Task Creation'),
      ),
      floatingActionButton: FloatingActionButton(
        child: const Icon(Icons.send),
        onPressed: () async {
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
                        await dispatchTask();
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
                  items: <String>['Patrol', 'Delivery'].map<DropdownMenuItem<String>>((String value) {
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
          Expanded(
            child: dropdownValue == 'Patrol'
                ? PatrolTaskCreationView(
                    controller: _taskCreationController,
                  )
                : DeliveryTaskCreationView(
                    controller: _taskCreationController,
                  ),
          ),
        ],
      ),
    );
  }
}
