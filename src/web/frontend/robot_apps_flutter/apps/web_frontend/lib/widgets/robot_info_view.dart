import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/widgets/modules_overview.dart';
import 'package:web_frontend/widgets/robot_task_view.dart';

class RobotInfoView extends StatelessWidget {
  const RobotInfoView({required this.robotName, super.key});
  final String robotName;
  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const SizedBox(height: 8),
          const Padding(
            padding: EdgeInsets.only(left: 16),
            child: Text(
              'Aufträge',
              style: TextStyle(fontSize: 32, color: WebColors.primaryText),
            ),
          ),
          RobotTaskView(robotName: robotName),
          const SizedBox(height: 8),
          const Padding(
            padding: EdgeInsets.only(left: 16),
            child: Text(
              'Module',
              style: TextStyle(fontSize: 32, color: WebColors.primaryText),
            ),
          ),
          const SizedBox(height: 4),
          ConstrainedBox(constraints: const BoxConstraints(maxHeight: 900), child: ModulesOverview(robotName: robotName)),
        ],
      ),
    );
  }
}
