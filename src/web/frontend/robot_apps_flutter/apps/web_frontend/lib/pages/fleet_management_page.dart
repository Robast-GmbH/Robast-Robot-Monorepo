import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';
import 'package:web_frontend/models/provider/robot_provider.dart';
import 'package:web_frontend/pages/task_creation_page.dart';
import 'package:web_frontend/widgets/robot_list_view.dart';

class FleetManagementPage extends StatefulWidget {
  const FleetManagementPage({super.key});

  @override
  State<FleetManagementPage> createState() => _FleetManagementPageState();
}

class _FleetManagementPageState extends State<FleetManagementPage> {
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
            MaterialPageRoute<TaskCreationPage>(
              builder: (context) => const TaskCreationPage(),
            ),
          );
        },
        child: const Icon(Icons.add_task),
      ),
      body: const RobotListView(),
    );
  }
}
