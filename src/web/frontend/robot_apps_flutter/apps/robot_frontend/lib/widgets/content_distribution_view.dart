import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_selector.dart';

class ContentDistributionView extends StatefulWidget {
  const ContentDistributionView({
    required this.preselectedSubmodules,
    required this.userSelectionControllers,
    required this.userGroupsSelectionControllers,
    required this.locationSelectionControllers,
    super.key,
  }) : assert(
          preselectedSubmodules.length == userSelectionControllers.length &&
              preselectedSubmodules.length == locationSelectionControllers.length &&
              preselectedSubmodules.length == userGroupsSelectionControllers.length,
          'preselectedSubmodules, userSelectionControllers, userGroupsSelectionControllers and locationSelectionControllers must have the same length',
        );

  final List<SubmoduleAddress> preselectedSubmodules;
  final List<UserSelectionController> userSelectionControllers;
  final List<UserGroupsSelectionController> userGroupsSelectionControllers;
  final List<LocationSelectionController> locationSelectionControllers;

  @override
  State<ContentDistributionView> createState() => _ContentDistributionViewState();
}

class _ContentDistributionViewState extends State<ContentDistributionView> {
  bool validateContentToTargetAssignments() {
    for (var i = 0; i < widget.preselectedSubmodules.length; i++) {
      final userSelectionController = widget.userSelectionControllers[i];
      final userGroupsSelectionController = widget.userGroupsSelectionControllers[i];
      if (userSelectionController.selectedUser == null && userGroupsSelectionController.selectionAsStringList().isEmpty) {
        return false;
      }
      final locationController = widget.locationSelectionControllers[i];
      if (locationController.room == null) {
        return false;
      }
    }
    return true;
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(top: 32, left: 32, right: 32),
      child: Column(
        children: [
          Expanded(
            child: Selector<ModuleProvider, List<Submodule>>(
              selector: (context, provider) => provider.submodules,
              builder: (context, submodules, child) {
                final selectedSubmodules = submodules
                    .where(
                      (submodule) => widget.preselectedSubmodules.any((preselectedSubmodule) => preselectedSubmodule == submodule.address),
                    )
                    .toList();
                return ListView(
                  children: List.generate(widget.preselectedSubmodules.length, (index) {
                    final submodule = selectedSubmodules[index];
                    return Card(
                      color: Colors.white.withOpacity(0.4),
                      child: Padding(
                        padding: const EdgeInsets.all(8),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(left: 8, bottom: 4),
                              child: Text(
                                'Modul ${submodule.address.moduleID}',
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
                                        submodule.contentToString(),
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
                                    onChanged: () => setState(() {}),
                                  ),
                                ),
                              ],
                            ),
                            Row(
                              children: [
                                Expanded(
                                  child: UserSelector(
                                    controller: widget.userSelectionControllers[index],
                                    onChanged: () => setState(() {}),
                                  ),
                                ),
                                Expanded(
                                  child: UserGroupsSelector(
                                    controller: widget.userGroupsSelectionControllers[index],
                                    onChanged: () => setState(() {}),
                                  ),
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
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.end,
            children: [
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 32),
                child: ElevatedButton(
                  onPressed: validateContentToTargetAssignments()
                      ? () async {
                          final submodules = Provider.of<ModuleProvider>(context, listen: false).submodules;
                          final taskProvider = Provider.of<TaskProvider>(context, listen: false);
                          for (var i = 0; i < widget.preselectedSubmodules.length; i++) {
                            final submoduleAddress = widget.preselectedSubmodules[i];
                            final submodule = submodules.firstWhere((element) => element.address == submoduleAddress);
                            final controller = widget.userSelectionControllers[i];
                            final user = controller.selectedUser;
                            final userGroups = widget.userGroupsSelectionControllers[i].selectionAsStringList();
                            await taskProvider.createDirectDropoffTask(
                              robotName: 'rb_theron',
                              dropoffTargetID: widget.locationSelectionControllers[i].room!,
                              user: user,
                              userGroups: userGroups,
                              submodule: submodule,
                            );
                          }
                          if (context.mounted) {
                            Navigator.pop(context);
                          }
                        }
                      : null,
                  child: const Padding(
                    padding: EdgeInsets.symmetric(vertical: 8, horizontal: 16),
                    child: Text(
                      'Fertig',
                      style: TextStyle(fontSize: 40),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}
