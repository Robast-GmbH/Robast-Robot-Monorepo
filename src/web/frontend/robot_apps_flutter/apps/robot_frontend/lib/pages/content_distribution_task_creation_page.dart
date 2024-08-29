import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/enums/creation_steps.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/content_distribution_view.dart';
import 'package:robot_frontend/widgets/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/modules_overview.dart';
import 'package:robot_frontend/widgets/reservation_view.dart';

class ContentDistributionTaskCreationPage extends StatefulWidget {
  const ContentDistributionTaskCreationPage({super.key});

  @override
  State<ContentDistributionTaskCreationPage> createState() => _ContentDistributionTaskCreationPageState();
}

class _ContentDistributionTaskCreationPageState extends State<ContentDistributionTaskCreationPage> {
  late Future<void> initPageFuture;
  late User? currentUser;
  CreationSteps currentStep = CreationSteps.reserveSubmodules;
  final reservedSubmodules = <SubmoduleAddress>[];
  final userSelectionControllers = <UserSelectionController>[];
  final userGroupsSelectionControllers = <UserGroupsSelectionController>[];
  final locationSelectionControllers = <LocationSelectionController>[];

  Future<void> initPage() async {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    final userProvider = Provider.of<UserProvider>(context, listen: false);
    final user = await userProvider.getUserSession(robotName: 'rb_theron');
    if (user != null) {
      final availableReservedSubmodules = moduleProvider.submodules.where(
        (submodule) => submodule.reservedForTask.isEmpty && submodule.checkUserAuth(user),
      );
      for (final submodule in availableReservedSubmodules) {
        onReservation(submodule.address);
      }
    }
    currentUser = user;
  }

  void onReservation(SubmoduleAddress submoduleAddress) {
    reservedSubmodules.add(submoduleAddress);
    userSelectionControllers.add(UserSelectionController());
    userGroupsSelectionControllers.add(UserGroupsSelectionController());
    locationSelectionControllers.add(LocationSelectionController());
  }

  void onFreeing(SubmoduleAddress submoduleAddress) {
    final index = reservedSubmodules.indexOf(submoduleAddress);
    reservedSubmodules.remove(submoduleAddress);
    userSelectionControllers.removeAt(index);
    userGroupsSelectionControllers.removeAt(index);
    locationSelectionControllers.removeAt(index);
  }

  @override
  void initState() {
    super.initState();
    initPageFuture = initPage();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: currentStep.name,
      onBackButtonPressed: () {
        if (currentStep == CreationSteps.reserveSubmodules) {
          Navigator.pop(context);
        } else if (currentStep == CreationSteps.fillModules) {
          setState(() {
            currentStep = CreationSteps.reserveSubmodules;
          });
        } else {
          setState(() {
            currentStep = CreationSteps.fillModules;
          });
        }
      },
      child: FutureBuilder<void>(
        future: initPageFuture,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(
              child: CircularProgressIndicator(),
            );
          }
          if (currentUser == null) {
            return Center(
              child: Column(
                children: [
                  const Text('Aktueller Nutzer konnte nicht geladen werden'),
                  TextButton(
                    onPressed: () {
                      initPageFuture = initPage();
                      setState(() {});
                    },
                    child: const Text('Erneut versuchen'),
                  ),
                ],
              ),
            );
          }
          return Stack(
            children: [
              if (currentStep == CreationSteps.reserveSubmodules)
                ReservationView(
                  currentUser: currentUser!,
                  onReservation: (submoduleAddress) {
                    onReservation(submoduleAddress);
                    setState(() {});
                  },
                  onFreeing: (submoduleAddress) {
                    onFreeing(submoduleAddress);
                    setState(() {});
                  },
                )
              else if (currentStep == CreationSteps.fillModules)
                Selector<ModuleProvider, List<Submodule>>(
                  selector: (context, provider) => provider.submodules,
                  builder: (context, submodules, child) {
                    return ModulesOverview(
                      submodules: submodules
                          .where((submodule) => reservedSubmodules.any((reservedSubmoduleAddress) => submodule.address == reservedSubmoduleAddress))
                          .toList(),
                    );
                  },
                )
              else if (currentStep == CreationSteps.assignTargets)
                ContentDistributionView(
                  preselectedSubmodules: reservedSubmodules,
                  userSelectionControllers: userSelectionControllers,
                  userGroupsSelectionControllers: userGroupsSelectionControllers,
                  locationSelectionControllers: locationSelectionControllers,
                ),
              if (currentStep != CreationSteps.assignTargets)
                Align(
                  alignment: Alignment.bottomRight,
                  child: Padding(
                    padding: const EdgeInsets.all(32),
                    child: CustomElevatedButton(
                      enabled: reservedSubmodules.isNotEmpty,
                      onPressed: () async {
                        if (currentStep == CreationSteps.reserveSubmodules) {
                          setState(() {
                            currentStep = CreationSteps.fillModules;
                          });
                        } else if (currentStep == CreationSteps.fillModules) {
                          setState(() {
                            currentStep = CreationSteps.assignTargets;
                          });
                        }
                      },
                      label: 'Weiter',
                    ),
                  ),
                ),
            ],
          );
        },
      ),
    );
  }
}
