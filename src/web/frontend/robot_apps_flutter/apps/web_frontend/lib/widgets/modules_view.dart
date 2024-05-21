import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/robot_provider.dart';

class ModulesView extends StatefulWidget {
  const ModulesView({required this.robotName, super.key});

  final String robotName;

  @override
  State<ModulesView> createState() => _ModulesViewState();
}

class _ModulesViewState extends State<ModulesView> {
  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicModuleUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicModuleUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Selector<RobotProvider, Map<String, List<DrawerModule>>>(
      selector: (_, provider) => provider.modules,
      builder: (context, modules, child) {
        final robotModules = modules[widget.robotName];
        return ListView.separated(
          itemCount: robotModules?.length ?? 0,
          separatorBuilder: (context, index) => const Divider(color: Colors.grey),
          itemBuilder: (context, index) {
            return ListTile(
              title: Text(robotModules![index].label),
              subtitle: Text(robotModules[index].isOpen ? 'Opened' : 'Closed'),
              style: ListTileStyle.list,
              onTap: () {
                if (robotModules[index].isOpen && robotModules[index].type == ModuleType.electric_drawer) {
                  Provider.of<RobotProvider>(context, listen: false).closeDrawer(
                    robotName: widget.robotName,
                    moduleID: robotModules[index].moduleID,
                    drawerID: robotModules[index].drawerID,
                  );
                } else if (!robotModules[index].isOpen) {
                  Provider.of<RobotProvider>(context, listen: false).openDrawer(
                    robotName: widget.robotName,
                    moduleID: robotModules[index].moduleID,
                    drawerID: robotModules[index].drawerID,
                  );
                }
              },
            );
          },
        );
      },
    );
  }
}
