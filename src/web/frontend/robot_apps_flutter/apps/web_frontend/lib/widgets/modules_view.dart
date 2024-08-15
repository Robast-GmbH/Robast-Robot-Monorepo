import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';

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
    Provider.of<FleetProvider>(context, listen: false).startPeriodicModuleUpdate();
  }

  @override
  void deactivate() {
    Provider.of<FleetProvider>(context, listen: false).stopPeriodicModuleUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Selector<FleetProvider, Map<String, List<RobotDrawer>>>(
      selector: (_, provider) => provider.modules,
      builder: (context, modules, child) {
        final robotModules = modules[widget.robotName];
        return ListView.separated(
          itemCount: robotModules?.length ?? 0,
          separatorBuilder: (context, index) => const Divider(color: Colors.grey),
          itemBuilder: (context, index) {
            final isOpen = robotModules![index].moduleProcess.status == ModuleProcessStatus.open;
            return ListTile(
              title: Text('${robotModules[index].address.moduleID}_${robotModules[index].address.drawerID}'),
              subtitle: Text(isOpen ? 'Opened' : 'Closed'),
              style: ListTileStyle.list,
              onTap: () {
                if (isOpen && robotModules[index].variant == DrawerVariant.electric) {
                  Provider.of<FleetProvider>(context, listen: false).closeDrawer(
                    robotName: widget.robotName,
                    moduleID: robotModules[index].address.moduleID,
                    drawerID: robotModules[index].address.drawerID,
                  );
                } else if (!isOpen) {
                  Provider.of<FleetProvider>(context, listen: false).openDrawer(
                    robotName: widget.robotName,
                    moduleID: robotModules[index].address.moduleID,
                    drawerID: robotModules[index].address.drawerID,
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
