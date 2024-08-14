import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
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
  final reservedDrawers = <DrawerAddress>[];
  final userSelectionController = <UserSelectionController>[];
  final locationSelectionControllers = <LocationSelectionController>[];

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: currentStep.name,
      child: Stack(
        children: [
          if (currentStep == CreationSteps.reserveModules)
            ReservationView(
              onReservation: (drawer) {
                reservedDrawers.add(drawer);
                userSelectionController.add(UserSelectionController());
                locationSelectionControllers.add(LocationSelectionController());
                setState(() {});
              },
            )
          else if (currentStep == CreationSteps.fillModules)
            ModuleFillingView(
              preselectedDrawers: reservedDrawers,
            )
          else
            ContentDistributionView(
              preselectedDrawers: reservedDrawers,
              userSelectionControllers: userSelectionController,
              locationSelectionControllers: locationSelectionControllers,
            ),
          Align(
            alignment: Alignment.bottomRight,
            child: Padding(
              padding: const EdgeInsets.all(32),
              child: ElevatedButton(
                onPressed: reservedDrawers.isEmpty
                    ? null
                    : () async {
                        if (currentStep == CreationSteps.reserveModules) {
                          setState(() {
                            currentStep = CreationSteps.fillModules;
                            // nicht befüllte Module wieder freigeben?
                          });
                        } else if (currentStep == CreationSteps.fillModules) {
                          setState(() {
                            currentStep = CreationSteps.assignTargets;
                          });
                        } else {
                          final modules = Provider.of<ModuleProvider>(context, listen: false).modules;
                          final taskProvider = Provider.of<TaskProvider>(context, listen: false);
                          for (var i = 0; i < reservedDrawers.length; i++) {
                            final drawerAddress = reservedDrawers[i];
                            final drawer = modules.firstWhere((element) => element.address == drawerAddress);
                            final controller = userSelectionController[i];
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
