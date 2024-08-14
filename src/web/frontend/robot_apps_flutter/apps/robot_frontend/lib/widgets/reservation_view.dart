import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/module_view.dart';

class ReservationView extends StatefulWidget {
  const ReservationView({
    required this.currentUser,
    required this.onReservation,
    required this.onFreeing,
    super.key,
  });

  final User currentUser;
  final void Function(DrawerAddress) onReservation;
  final void Function(DrawerAddress) onFreeing;

  @override
  State<ReservationView> createState() => _ReservationViewState();
}

class _ReservationViewState extends State<ReservationView> {
  bool isUpdatingReservationStatus = false;

  @override
  Widget build(BuildContext context) {
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
                                  userIDs: [widget.currentUser.id],
                                  userGroups: widget.currentUser.userGroups,
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
                              } else if (submodule.isReserved() && submodule.checkUserAuth(widget.currentUser) && !isUpdatingReservationStatus) {
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
  }
}
