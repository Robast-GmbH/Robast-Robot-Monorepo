import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/titled_view.dart';

class TaskRequestCreationView extends StatefulWidget {
  const TaskRequestCreationView({super.key});

  @override
  State<TaskRequestCreationView> createState() => _TaskRequestCreationViewState();
}

class _TaskRequestCreationViewState extends State<TaskRequestCreationView> {
  @override
  Widget build(BuildContext context) {
    return const TitledView(title: 'Auftrag erstellen', child: SizedBox());
  }
}
