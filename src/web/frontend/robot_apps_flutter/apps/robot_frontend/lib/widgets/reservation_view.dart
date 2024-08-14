import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/drawer_view.dart';

class ReservationView extends StatefulWidget {
  const ReservationView({this.onReservation, super.key});

  final void Function(DrawerAddress)? onReservation;

  @override
  State<ReservationView> createState() => _ReservationViewState();
}

class _ReservationViewState extends State<ReservationView> {
  late final Future<User?> loadCurrentUserFuture;
  bool isReserving = false;

  @override
  void initState() {
    super.initState();
    loadCurrentUserFuture = Provider.of<UserProvider>(context, listen: false).getUserSession(robotName: 'rb_theron');
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<User?>(
      future: loadCurrentUserFuture,
      builder: (context, snapshot) {
        if (snapshot.connectionState != ConnectionState.done) {
          return const Center(child: CircularProgressIndicator());
        }
        return Selector<ModuleProvider, List<RobotDrawer>>(
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
                        children: modules
                            .map(
                              (drawer) => DrawerView(
                                module: drawer,
                                label: 'Modul ${drawer.address.moduleID}',
                                showReservationStatus: true,
                                onPressed: () async {
                                  if (!drawer.isReserved() && snapshot.data != null && !isReserving) {
                                    isReserving = true;
                                    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
                                    final reservationSuccessful = await moduleProvider.reserveModule(
                                      drawerAddress: drawer.address,
                                      userIDs: [snapshot.data!.id],
                                      userGroups: snapshot.data!.userGroups,
                                    );
                                    await moduleProvider.fetchModules();
                                    isReserving = false;
                                    widget.onReservation?.call(
                                      drawer.address,
                                    );
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
