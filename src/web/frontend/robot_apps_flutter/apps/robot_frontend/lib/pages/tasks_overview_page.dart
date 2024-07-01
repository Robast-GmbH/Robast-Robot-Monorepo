import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class TasksOverviewPage extends StatelessWidget {
  const TasksOverviewPage({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      title: 'Auftragsübersicht',
      child: Column(
        children: [
          Expanded(
            child: Center(
              child: Text(
                'Moin',
                style: TextStyle(fontSize: 80),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
