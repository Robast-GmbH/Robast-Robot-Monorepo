import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/pages/robot_page.dart';
import 'package:web_frontend/widgets/next_cleaning_view.dart';

class RobotListView extends StatelessWidget {
  const RobotListView({super.key});

  @override
  Widget build(BuildContext context) {
    return Selector<FleetProvider, List<Robot>>(
      selector: (_, robotProvider) => robotProvider.robots,
      builder: (context, robots, child) {
        return ListView.builder(
          itemCount: robots.length,
          itemBuilder: (context, index) {
            return Card(
              margin: const EdgeInsets.all(16),
              child: ListTile(
                title: Text(robots[index].name),
                subtitle: NextCleaningView(robotName: robots[index].name),
                onTap: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute<RobotPage>(
                      builder: (context) => RobotPage(robot: robots[index]),
                    ),
                  );
                },
              ),
            );
          },
        );
      },
    );
  }
}
