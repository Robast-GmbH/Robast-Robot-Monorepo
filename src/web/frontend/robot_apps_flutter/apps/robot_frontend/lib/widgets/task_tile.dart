import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/services/time_stamp_formatter.dart';
import 'package:robot_frontend/widgets/expandable_subtask_tile.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class TaskTile extends StatelessWidget {
  const TaskTile({required this.task, super.key});

  final Task task;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: RoundedContainer(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: Row(
                children: [
                  Text(
                    task.name,
                    style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
                  ),
                  const SizedBox(
                    width: 8,
                  ),
                  const Text(
                    'erstellt am',
                    style: TextStyle(fontSize: 32, color: RobotColors.primaryText),
                  ),
                  const SizedBox(
                    width: 8,
                  ),
                  Text(
                    TimeStampFormatter.format(unixTimeStamp: task.earliestStartTime),
                    style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
                  )
                ],
              ),
            ),
            Column(
              children: task.subtasks.map((subtask) {
                return Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: ExpandableSubtaskTile(subtask: subtask),
                );
              }).toList(),
            ),
          ],
        ),
      ),
    );
  }
}
