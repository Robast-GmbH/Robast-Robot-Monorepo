import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/pages/config_page.dart';
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
    Provider.of<FleetProvider>(context, listen: false).updateProviderData();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Fleet Management'),
        actions: [
          IconButton(
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute<ConfigPage>(
                    builder: (context) => const ConfigPage(),
                  ),
                );
              },
              icon: const Icon(Icons.settings),),
        ],
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
