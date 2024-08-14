import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class ExpandableTaskTile extends StatefulWidget {
  const ExpandableTaskTile({
    required this.task,
    super.key,
  });

  final SubTask task;

  @override
  State<ExpandableTaskTile> createState() => _ExpandableTaskTileState();
}

class _ExpandableTaskTileState extends State<ExpandableTaskTile> {
  bool isExpanded = false;

  String unixTimeStampToFormattedString({required int unixTimeStamp}) {
    final date = DateTime.fromMillisecondsSinceEpoch(unixTimeStamp * 1000);
    return '${date.day}.${date.month}.${date.year} um ${date.hour}:${date.minute < 10 ? '0${date.minute}' : date.minute}:${date.second < 10 ? '0${date.second}' : date.second} Uhr';
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Theme(
          data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
          child: ExpansionTile(
            title: Row(
              children: [
                const SizedBox(
                  width: 4,
                ),
                Text('at ${widget.task.targetID}'),
                const SizedBox(
                  width: 4,
                ),
              ],
            ),
            children: [
              buildTaskInfoTile(title: 'ID', subtitle: widget.task.id),
              buildTaskInfoTile(title: 'Status', subtitle: widget.task.status),
              if (widget.task.requiresTaskID != null) buildTaskInfoTile(title: 'Erfordert Task', subtitle: widget.task.requiresTaskID!),
              buildTaskInfoTile(title: 'Zugewiesen an', subtitle: widget.task.assigneeName),
              buildTaskInfoTile(title: 'Priorität', subtitle: widget.task.priority.toString()),
            ],
          ),
        ),
      ),
    );
  }

  ListTile buildTaskInfoTile({
    required String title,
    required String subtitle,
  }) {
    return ListTile(
      title: Text(title),
      subtitle: Text(subtitle),
    );
  }
}
