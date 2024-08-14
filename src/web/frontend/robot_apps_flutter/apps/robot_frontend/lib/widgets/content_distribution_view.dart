import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';

class ContentDistributionView extends StatefulWidget {
  const ContentDistributionView({
    required this.preselectedDrawers,
    required this.userSelectionControllers,
    required this.locationSelectionControllers,
    super.key,
  }) : assert(
          preselectedDrawers.length == userSelectionControllers.length && preselectedDrawers.length == locationSelectionControllers.length,
          'preselectedDrawers, userSelectionControllers and locationSelectionControllers must have the same length',
        );

  final List<DrawerAddress> preselectedDrawers;
  final List<UserSelectionController> userSelectionControllers;
  final List<LocationSelectionController> locationSelectionControllers;

  @override
  State<ContentDistributionView> createState() => _ContentDistributionViewState();
}

class _ContentDistributionViewState extends State<ContentDistributionView> {
  @override
  Widget build(BuildContext context) {
    return Selector<ModuleProvider, List<RobotDrawer>>(
      selector: (context, provider) => provider.modules,
      builder: (context, modules, child) {
        final selectedDrawers = modules
            .where(
              (drawer) => widget.preselectedDrawers.any((preselectedDrawer) => preselectedDrawer == drawer.address),
            )
            .toList();
        return ListView(
          children: List.generate(widget.preselectedDrawers.length, (index) {
            final drawer = selectedDrawers[index];
            return Row(
              children: [
                Text('Modul ${drawer.address.moduleID}'),
                Text(drawer.itemsByCount.toString()),
                Expanded(
                  child: UserSelector(
                    controller: widget.userSelectionControllers[index],
                  ),
                ),
                Expanded(
                  child: LocationSelector(
                    controller: widget.locationSelectionControllers[index],
                    label: 'Zielort',
                  ),
                )
              ],
            );
          }).toList(),
        );
      },
    );
  }
}
