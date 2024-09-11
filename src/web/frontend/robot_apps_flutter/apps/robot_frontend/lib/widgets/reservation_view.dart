import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
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
  final void Function(SubmoduleAddress) onReservation;
  final void Function(SubmoduleAddress) onFreeing;

  @override
  State<ReservationView> createState() => _ReservationViewState();
}

class _ReservationViewState extends State<ReservationView> {
  bool isUpdatingReservationStatus = false;

  Future<void> onReservation(Submodule submodule) async {
    if (!submodule.isReserved() && !isUpdatingReservationStatus) {
      isUpdatingReservationStatus = true;
      final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
      final reservationSuccessful = await moduleProvider.reserveSubmodule(
        submoduleAddress: submodule.address,
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
  }

  @override
  Widget build(BuildContext context) {
    return Selector<ModuleProvider, List<List<Submodule>>>(
      selector: (context, provider) => provider.modules,
      builder: (context, modules, child) {
        if (modules.isNotEmpty) {
          return Row(
            children: [
              const Expanded(child: SizedBox()),
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.only(top: 32, bottom: 64),
                  child: Column(
                    children: List.generate(
                      modules.length,
                      (index) {
                        return Expanded(
                          flex: modules[index].first.size,
                          child: Stack(
                            alignment: Alignment.center,
                            fit: StackFit.expand,
                            children: [
                              Row(
                                children: modules[index]
                                    .map(
                                      (submodule) => ModuleView(
                                        module: submodule,
                                        showReservationStatus: true,
                                        enabled: submodule.reservedForTask.isEmpty,
                                        onPressed: () => onReservation(submodule),
                                      ),
                                    )
                                    .toList(),
                              ),
                              IgnorePointer(
                                child: Align(
                                  alignment: Alignment.center,
                                  child: Text(
                                    'Modul ${index + 1}',
                                    textAlign: TextAlign.center,
                                    style: TextStyle(
                                      height: 0,
                                      color: modules[index].any((submodule) => submodule.reservedForTask.isEmpty)
                                          ? RobotColors.secondaryText
                                          : RobotColors.primaryText.withOpacity(0.2),
                                      fontSize: 40,
                                      fontWeight: FontWeight.w400,
                                    ),
                                  ),
                                ),
                              ),
                            ],
                          ),
                        );
                      },
                    ).toList(),
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
