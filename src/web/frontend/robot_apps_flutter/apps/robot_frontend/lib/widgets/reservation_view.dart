import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/module_view.dart';

class ReservationView extends StatefulWidget {
  const ReservationView({
    required this.onReservation,
    required this.onFreeing,
    super.key,
  });

  final void Function(DrawerAddress) onReservation;
  final void Function(DrawerAddress) onFreeing;

  @override
  State<ReservationView> createState() => _ReservationViewState();
}

class _ReservationViewState extends State<ReservationView> {
  late final Future<User?> initReservationViewFuture;
  bool isUpdatingReservationStatus = false;

  Future<User?> initReservationView() async {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    final userProvider = Provider.of<UserProvider>(context, listen: false);
    final user = await userProvider.getUserSession(robotName: 'rb_theron');
    if (user != null) {
      final availableReservedSubmodules = moduleProvider.submodules.where(
        (submodule) => submodule.reservedForTask.isEmpty && submodule.checkUserAuth(user),
      );
      for (final submodule in availableReservedSubmodules) {
        widget.onReservation(submodule.address);
      }
    }
    return user;
  }

  @override
  void initState() {
    super.initState();
    initReservationViewFuture = initReservationView();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<User?>(
      future: initReservationViewFuture,
      builder: (context, snapshot) {
        if (snapshot.connectionState != ConnectionState.done || snapshot.data == null) {
          return const Center(child: CircularProgressIndicator());
        }
        final user = snapshot.data!;
        return Selector<ModuleProvider, List<RobotDrawer>>(
          selector: (context, provider) => provider.submodules,
          builder: (context, submodules, child) {
            if (submodules.isNotEmpty) {
              return Row(
                children: [
                  const Expanded(child: SizedBox()),
                  Expanded(
                    child: Padding(
                      padding: const EdgeInsets.only(top: 32, bottom: 64),
                      child: Column(
                        children: submodules
                            .map(
                              (submodule) => ModuleView(
                                module: submodule,
                                label: 'Modul ${submodule.address.moduleID}',
                                showReservationStatus: true,
                                enabled: submodule.reservedForTask.isEmpty,
                                onPressed: () async {
                                  if (!submodule.isReserved() && !isUpdatingReservationStatus) {
                                    isUpdatingReservationStatus = true;
                                    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
                                    final reservationSuccessful = await moduleProvider.reserveModule(
                                      drawerAddress: submodule.address,
                                      userIDs: [user.id],
                                      userGroups: user.userGroups,
                                    );
                                    await moduleProvider.fetchModules();
                                    isUpdatingReservationStatus = false;
                                    widget.onReservation(submodule.address);
                                    if (!reservationSuccessful && context.mounted) {
                                      await showDialog<AlertDialog>(
                                        context: context,
                                        builder: (context) {
                                          return AlertDialog(
                                            title: const Text('Modul Reservierung fehlgeschlagen'),
                                            content: const Text('Das Modul ist bereits reserviert'),
                                            actions: [
                                              TextButton(
                                                onPressed: () => Navigator.of(context).pop(),
                                                child: const Text('OK'),
                                              ),
                                            ],
                                          );
                                        },
                                      );
                                    }
                                  } else if (submodule.isReserved() && submodule.checkUserAuth(user) && !isUpdatingReservationStatus) {
                                    isUpdatingReservationStatus = true;
                                    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
                                    final freeingSuccessful = await moduleProvider.freeSubmodule(
                                      submoduleAddress: submodule.address,
                                    );
                                    await moduleProvider.fetchModules();
                                    isUpdatingReservationStatus = false;
                                    widget.onFreeing(submodule.address);
                                    if (!freeingSuccessful && context.mounted) {
                                      await showDialog<AlertDialog>(
                                        context: context,
                                        builder: (context) {
                                          return AlertDialog(
                                            title: const Text('Modul Freigabe fehlgeschlagen'),
                                            content: const Text('Das Modul ist bereits freigegeben'),
                                            actions: [
                                              TextButton(
                                                onPressed: () => Navigator.of(context).pop(),
                                                child: const Text('OK'),
                                              ),
                                            ],
                                          );
                                        },
                                      );
                                    }
                                  }
                                },
                              ),
                            )
                            .toList(),
                      ),
                    ),
                  ),
                  const Expanded(child: SizedBox()),
                ],
              );
            }
            return const Center(child: CircularProgressIndicator());
          },
        );
      },
    );
  }
}
