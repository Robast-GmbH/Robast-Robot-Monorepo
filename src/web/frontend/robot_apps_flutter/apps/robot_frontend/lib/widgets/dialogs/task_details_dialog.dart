import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class TaskDetailsDialog extends StatelessWidget {
  const TaskDetailsDialog({required this.task, super.key});

  final SubTask task;

  @override
  Widget build(BuildContext context) {
    final actionSequence = <RobotAction>[];
    RobotAction? action = task.action;
    while (action != null) {
      actionSequence.add(action);
      action = action.subaction;
    }
    return AlertDialog(
      backgroundColor: RobotColors.primaryBackground,
      title: Text(
        '${task.name} bei ${task.targetID}',
        style: const TextStyle(fontSize: 40, color: RobotColors.primaryText),
      ),
      content: FractionallySizedBox(
        heightFactor: 0.6,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Status: ${task.status}',
              style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
            ),
            const SizedBox(
              height: 4,
            ),
            const Text(
              'Aktionen',
              style: TextStyle(fontSize: 32, color: RobotColors.primaryText),
            ),
            const SizedBox(
              height: 4,
            ),
            Expanded(
              child: SingleChildScrollView(
                child: Column(
                    children: actionSequence.map((action) {
                  final submoduleAddress = action.parameters['submodule_address'];
                  return RoundedContainer(
                    child: Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            'Typ: ${action.name}',
                            style: const TextStyle(fontSize: 28, color: RobotColors.secondaryText),
                          ),
                          const SizedBox(
                            height: 4,
                          ),
                          Text(
                            'Status: ${action.status}',
                            style: const TextStyle(fontSize: 28, color: RobotColors.secondaryText),
                          ),
                          const SizedBox(
                            height: 4,
                          ),
                          RoundedContainer(
                            child: Padding(
                              padding: const EdgeInsets.all(8.0),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.stretch,
                                children: [
                                  const Text('Parameter:', style: TextStyle(fontSize: 28, color: RobotColors.secondaryText)),
                                  Text(
                                    'Modul ${submoduleAddress['module_id']} Submodul ${submoduleAddress['submodule_id']}',
                                    style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                  ),
                                  ...(action.parameters['items_by_change'] as Map<String, dynamic>).keys.map((key) {
                                    final change = action.parameters['items_by_change'][key] as int;
                                    return Text(
                                      '${change > 0 ? 'Beladen mit ' : ''}${change.abs()} $key${change < 0 ? ' entladen' : ''}',
                                      style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                    );
                                  }),
                                ],
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  );
                }).toList()),
              ),
            ),
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.pop(context);
          },
          child: const Text(
            'Schließen',
            style: TextStyle(color: RobotColors.secondaryText, fontSize: 32),
          ),
        ),
      ],
    );
  }
}
