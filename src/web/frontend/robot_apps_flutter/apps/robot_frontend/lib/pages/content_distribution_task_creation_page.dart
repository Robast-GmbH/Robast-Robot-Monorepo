import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/enums/creation_steps.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/content_distribution_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/module_filling_view.dart';
import 'package:robot_frontend/widgets/reservation_view.dart';

class ContentDistributionTaskCreationPage extends StatefulWidget {
  const ContentDistributionTaskCreationPage({super.key});

  @override
  State<ContentDistributionTaskCreationPage> createState() => _ContentDistributionTaskCreationPageState();
}

class _ContentDistributionTaskCreationPageState extends State<ContentDistributionTaskCreationPage> {
  CreationSteps currentStep = CreationSteps.reserveModules;
  final reservedSubmodules = <DrawerAddress>[];
  final userSelectionControllers = <UserSelectionController>[];
  final userGroupsSelectionControllers = <UserGroupsSelectionController>[];
  final locationSelectionControllers = <LocationSelectionController>[];

  bool validateTaskAssignments() {
    for (var i = 0; i < reservedSubmodules.length; i++) {
      final userSelectionController = userSelectionControllers[i];
      final userGroupsSelectionController = userGroupsSelectionControllers[i];
      if (userSelectionController.selectedUser == null && userGroupsSelectionController.selectionAsStringList().isEmpty) {
        return false;
      }
      final locationController = locationSelectionControllers[i];
      if (locationController.room == null) {
        return false;
      }
    }
    return true;
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: currentStep.name,
      child: Stack(
        children: [
          if (currentStep == CreationSteps.reserveModules)
            ReservationView(
              onReservation: (submoduleAddress) {
                reservedSubmodules.add(submoduleAddress);
                userSelectionControllers.add(UserSelectionController());
                userGroupsSelectionControllers.add(UserGroupsSelectionController());
                locationSelectionControllers.add(LocationSelectionController());
                setState(() {});
              },
              onFreeing: (submoduleAddress) {
                final index = reservedSubmodules.indexOf(submoduleAddress);
                reservedSubmodules.remove(submoduleAddress);
                userSelectionControllers.removeAt(index);
                userGroupsSelectionControllers.removeAt(index);
                locationSelectionControllers.removeAt(index);
                setState(() {});
              },
            )
          else if (currentStep == CreationSteps.fillModules)
            ModuleFillingView(
              preselectedDrawers: reservedSubmodules,
            )
          else
            ContentDistributionView(
              preselectedDrawers: reservedSubmodules,
              userSelectionControllers: userSelectionControllers,
              userGroupsSelectionControllers: userGroupsSelectionControllers,
              locationSelectionControllers: locationSelectionControllers,
            ),
          if (currentStep != CreationSteps.assignTargets)
            Align(
              alignment: Alignment.bottomRight,
              child: Padding(
                padding: const EdgeInsets.all(32),
                child: ElevatedButton(
                  onPressed: reservedSubmodules.isEmpty
                      ? null
                      : () async {
                          if (currentStep == CreationSteps.reserveModules) {
                            setState(() {
                              currentStep = CreationSteps.fillModules;
                            });
                          } else if (currentStep == CreationSteps.fillModules) {
                            setState(() {
                              currentStep = CreationSteps.assignTargets;
                            });
                          } else {
                            final modules = Provider.of<ModuleProvider>(context, listen: false).submodules;
                            final taskProvider = Provider.of<TaskProvider>(context, listen: false);
                            for (var i = 0; i < reservedSubmodules.length; i++) {
                              final drawerAddress = reservedSubmodules[i];
                              final drawer = modules.firstWhere((element) => element.address == drawerAddress);
                              final controller = userSelectionControllers[i];
                              final user = controller.selectedUser!;
                              await taskProvider.createDirectDropoffTask(
                                robotName: 'rb_theron',
                                dropoffTargetID: locationSelectionControllers[i].room!,
                                user: user,
                                drawer: drawer,
                              );
                            }
                            if (context.mounted) {
                              Navigator.pop(context);
                            }
                          }
                        },
                  child: const Padding(
                    padding: EdgeInsets.all(8),
                    child: Text(
                      'Weiter',
                      style: TextStyle(fontSize: 40),
                    ),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}
