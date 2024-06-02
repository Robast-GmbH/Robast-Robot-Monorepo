import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';
import 'package:web_frontend/models/provider/robot_provider.dart';
import 'package:web_frontend/views/task_creation_view.dart';
import 'package:web_frontend/widgets/robot_list_view.dart';

class FleetManagementView extends StatefulWidget {
  const FleetManagementView({super.key});

  @override
  State<FleetManagementView> createState() => _FleetManagementViewState();
}

class _FleetManagementViewState extends State<FleetManagementView> {
  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).updateProviderData();
    Provider.of<RMFProvider>(context, listen: false).updateBuildingMap();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Fleet Management'),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          Navigator.push(
            context,
            MaterialPageRoute<TaskCreationView>(
              builder: (context) => const TaskCreationView(),
            ),
          );
        },
        child: const Icon(Icons.add_task),
      ),
      body: const RobotListView(),
    );
  }
}
