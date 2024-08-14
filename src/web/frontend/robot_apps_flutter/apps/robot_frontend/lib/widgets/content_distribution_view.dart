import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';

class ContentDistributionView extends StatefulWidget {
  const ContentDistributionView({
    required this.preselectedDrawers,
    required this.userSelectionControllers,
    required this.userGroupsSelectionControllers,
    required this.locationSelectionControllers,
    super.key,
  }) : assert(
          preselectedDrawers.length == userSelectionControllers.length &&
              preselectedDrawers.length == locationSelectionControllers.length &&
              preselectedDrawers.length == userGroupsSelectionControllers.length,
          'preselectedDrawers, userSelectionControllers and locationSelectionControllers must have the same length',
        );

  final List<DrawerAddress> preselectedDrawers;
  final List<UserSelectionController> userSelectionControllers;
  final List<UserGroupsSelectionController> userGroupsSelectionControllers;
  final List<LocationSelectionController> locationSelectionControllers;

  @override
  State<ContentDistributionView> createState() => _ContentDistributionViewState();
}

class _ContentDistributionViewState extends State<ContentDistributionView> {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 32, horizontal: 64),
      child: Selector<ModuleProvider, List<RobotDrawer>>(
        selector: (context, provider) => provider.submodules,
        builder: (context, modules, child) {
          final selectedDrawers = modules
              .where(
                (drawer) => widget.preselectedDrawers.any((preselectedDrawer) => preselectedDrawer == drawer.address),
              )
              .toList();
          return ListView(
            children: List.generate(widget.preselectedDrawers.length, (index) {
              final drawer = selectedDrawers[index];
              return Card(
                color: Colors.white.withOpacity(0.4),
                child: Padding(
                  padding: const EdgeInsets.all(8),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Padding(
                        padding: const EdgeInsets.all(4),
                        child: Text(
                          'Modul ${drawer.address.moduleID}',
                          style: const TextStyle(fontSize: 32),
                        ),
                      ),
                      Card(
                        color: Colors.white.withOpacity(0.4),
                        child: Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                          child: Row(
                            children: [
                              const Text(
                                'Inhalte',
                                style: TextStyle(
                                  fontSize: 24,
                                ),
                              ),
                              Expanded(
                                child: Text(
                                  drawer.contentToString(),
                                  textAlign: TextAlign.end,
                                  style: const TextStyle(
                                    fontSize: 24,
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                      Row(
                        children: [
                          Expanded(
                            flex: 2,
                            child: LocationSelector(
                              controller: widget.locationSelectionControllers[index],
                              label: 'Zielort',
                            ),
                          ),
                        ],
                      ),
                      Row(
                        children: [
                          Expanded(
                            child: UserSelector(
                              controller: widget.userSelectionControllers[index],
                            ),
                          ),
                          Expanded(
                            child: UserGroupsSelector(controller: widget.userGroupsSelectionControllers[index]),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              );
            }).toList(),
          );
        },
      ),
    );
  }
}
