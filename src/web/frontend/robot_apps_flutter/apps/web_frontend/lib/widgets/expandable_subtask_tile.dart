import 'package:flutter/material.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/services/time_stamp_formatter.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class ExpandableSubtaskTile extends StatefulWidget {
  const ExpandableSubtaskTile({
    required this.subtask,
    super.key,
  });

  final SubTask subtask;

  @override
  State<ExpandableSubtaskTile> createState() => _ExpandableSubtaskTileState();
}

class _ExpandableSubtaskTileState extends State<ExpandableSubtaskTile> {
  bool isExpanded = false;

  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Theme(
          data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
          child: ExpansionTile(
            title: Text(
              '${widget.subtask.name} bei ${widget.subtask.targetID}',
              style: const TextStyle(fontSize: 22),
            ),
            children: [
              buildSubtaskaskInfoTile(title: 'Erstelldatum', subtitle: TimeStampFormatter.format(unixTimeStamp: widget.subtask.earliestStartTime)),
              buildSubtaskaskInfoTile(title: 'Status', subtitle: widget.subtask.status),
              buildSubtaskaskInfoTile(title: 'Aktion', subtitle: widget.subtask.action.parameters.toString()),
              buildSubtaskaskInfoTile(title: 'Priorität', subtitle: widget.subtask.priority.toString()),
            ],
          ),
        ),
      ),
    );
  }

  ListTile buildSubtaskaskInfoTile({
    required String title,
    required String subtitle,
  }) {
    return ListTile(
      title: Text(
        title,
        style: const TextStyle(fontSize: 20),
      ),
      subtitle: Text(subtitle),
    );
  }
}
