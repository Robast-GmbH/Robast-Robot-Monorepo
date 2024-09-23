import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/widgets/modules_overview.dart';
import 'package:web_frontend/widgets/robot_map_view.dart';
import 'package:web_frontend/widgets/robot_task_view.dart';

class RobotPage extends StatefulWidget {
  const RobotPage({required this.robot, super.key});

  final Robot robot;

  @override
  State<RobotPage> createState() => _RobotPageState();
}

class _RobotPageState extends State<RobotPage> {
  int _selectedIndex = 1;

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  Widget getWidgetOption(int index) {
    final widgetOptions = <Widget>[
      RobotTaskView(robotName: widget.robot.name),
      RobotMapView(robotName: widget.robot.name),
      ModulesOverview(robotName: widget.robot.name),
    ];
    return widgetOptions[index];
  }

  @override
  void initState() {
    super.initState();
    Provider.of<FleetProvider>(context, listen: false).startPeriodicModuleUpdate();
  }

  @override
  void deactivate() {
    Provider.of<FleetProvider>(context, listen: false).stopPeriodicModuleUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Übersicht: ${widget.robot.name}'),
      ),
      body: Center(
        child: getWidgetOption(_selectedIndex),
      ),
      bottomNavigationBar: BottomNavigationBar(
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(
            icon: Icon(Icons.assignment),
            label: 'Aufträge',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.map),
            label: 'Karte',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.extension),
            label: 'Module',
          ),
        ],
        currentIndex: _selectedIndex,
        selectedItemColor: Colors.amber[800],
        onTap: _onItemTapped,
      ),
    );
  }
}
