import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/auth_view.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';
import 'package:robot_frontend/widgets/module_process_views/closed_view.dart';
import 'package:robot_frontend/widgets/module_process_views/closing_view.dart';
import 'package:robot_frontend/widgets/module_process_views/open_view.dart';
import 'package:robot_frontend/widgets/module_process_views/opening_timed_out_view.dart';
import 'package:robot_frontend/widgets/module_process_views/opening_view.dart';
import 'package:robot_frontend/widgets/module_process_views/stallguard_triggered_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class ModuleProcessView extends StatefulWidget {
  const ModuleProcessView({
    this.requireDesinfection = false,
    super.key,
  });

  final bool requireDesinfection;

  @override
  State<ModuleProcessView> createState() => _ModuleProcessViewState();
}

class _ModuleProcessViewState extends State<ModuleProcessView> {
  bool isDisinfected = false;
  bool openingTriggered = false;
  bool reopeningTriggered = true;
  Timer? finishedTimer;
  Timer? openingFailedTimer;
  bool openingFailed = false;
  bool waitingForFinish = false;
  bool isDisplayingManuals = false;
  int finishTimerClockIndex = 0;

  @override
  void initState() {
    super.initState();
    isDisinfected = !widget.requireDesinfection;
  }

  void startFinishedTimer(Submodule moduleInProcess) {
    finishedTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      finishTimerClockIndex++;
      setState(() {});
      if (finishTimerClockIndex == 5) {
        onFinish(moduleInProcess);
      }
    });
  }

  Future<void> onFinish(Submodule moduleInProcess) async {
    if (!mounted) {
      return;
    }
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    await moduleProvider.finishSubmoduleProcess(moduleInProcess);
    await moduleProvider.fetchModules();
    finishedTimer?.cancel();
    if (mounted) {
      Navigator.pop(context);
    }
  }

  Future<void> onReopen(Submodule moduleInProcess) async {
    await Provider.of<ModuleProvider>(context, listen: false).openSubmodule(moduleInProcess);
    finishedTimer?.cancel();
    waitingForFinish = false;
    finishTimerClockIndex = 0;
    reopeningTriggered = true;
    setState(() {});
  }

  @override
  void dispose() {
    finishedTimer?.cancel();
    openingFailedTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      fit: StackFit.expand,
      children: [
        Selector<ModuleProvider, List<Submodule>>(
          selector: (_, provider) => provider.submodules,
          builder: (context, modules, child) {
            final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
            if ((modules.isEmpty || modules.every((module) => module.moduleProcess.status == ModuleProcessStatus.idle))) {
              if ((openingFailedTimer == null || !openingFailedTimer!.isActive) && !openingFailed) {
                openingFailedTimer = Timer(const Duration(seconds: 3), () {
                  openingFailed = true;
                  setState(() {});
                });
              }
              if (!openingFailed) {
                return const Center(
                  child: CircularProgressIndicator(),
                );
              } else {
                return Center(
                  child: TextButton(
                      onPressed: () {
                        Navigator.pop(context);
                      },
                      child: const Text(
                        "Öffnung fehlerhaft. Hier drücken um abzubrechen.",
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 40),
                      )),
                );
              }
            }
            openingFailedTimer?.cancel();
            final moduleInProcess = moduleProvider.submodules.firstWhere(
              (element) => element.moduleProcess.status != ModuleProcessStatus.idle,
            );
            final processStatus = moduleInProcess.moduleProcess.status;
            final position = moduleProvider.modules.indexWhere((submodules) => submodules.contains(moduleInProcess)) + 1;
            if (processStatus != ModuleProcessStatus.closed) {
              waitingForFinish = false;
              finishedTimer?.cancel();
              finishTimerClockIndex = 0;
              reopeningTriggered = false;
            }
            if (isDisinfected && processStatus == ModuleProcessStatus.waitingForOpening && !openingTriggered) {
              openingTriggered = true;
              WidgetsBinding.instance.addPostFrameCallback((_) {
                moduleProvider.openSubmodule(moduleInProcess);
              });
            }
            if (processStatus == ModuleProcessStatus.closed && !waitingForFinish && !reopeningTriggered && !isDisplayingManuals) {
              waitingForFinish = true;
              startFinishedTimer(moduleInProcess);
            }

            if (processStatus == ModuleProcessStatus.auth) {
              return AuthView(
                requiredUserIDs: moduleInProcess.reservedForIds,
                requiredUserGroups: moduleInProcess.reservedForGroups,
                onAuthCompleted: ({required bool wasSuccessful}) {
                  if (wasSuccessful) {
                    moduleProvider.fetchModules();
                  }
                },
              );
            }
            if (!isDisinfected) {
              return DisinfectionView(
                onDisinfection: () {
                  setState(() {
                    isDisinfected = true;
                  });
                },
              );
            }
            if (processStatus == ModuleProcessStatus.stallguardTriggered) {
              return StallguardTriggeredView(submodule: moduleInProcess);
            }
            if (processStatus == ModuleProcessStatus.openingTimedOut) {
              return OpeningTimedOutView(submodule: moduleInProcess);
            }
            if (processStatus == ModuleProcessStatus.opening || reopeningTriggered) {
              return OpeningView(
                submodule: moduleInProcess,
                position: position,
              );
            }
            if (processStatus == ModuleProcessStatus.open) {
              return OpenView(
                submodule: moduleInProcess,
                position: position,
              );
            }
            if (processStatus == ModuleProcessStatus.closing) {
              return ClosingView(position: position);
            }
            if (processStatus == ModuleProcessStatus.closed) {
              return ClosedView(
                  onFinish: () => onFinish(moduleInProcess),
                  onReopen: () => onReopen(moduleInProcess),
                  submodule: moduleInProcess,
                  secondsToFinish: finishTimerClockIndex);
            }
            return const SizedBox();
          },
        ),
        Align(
          alignment: Alignment.topLeft,
          child: Padding(
            padding: const EdgeInsets.only(left: 16.0, top: 8),
            child: IconButton(
              onPressed: () async {
                isDisplayingManuals = true;
                waitingForFinish = false;
                finishedTimer?.cancel();
                openingFailedTimer?.cancel();
                await Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (context) => const ManualsPage(
                      inactivityTimerEnabled: false,
                    ),
                  ),
                );
                isDisplayingManuals = false;
              },
              icon: const Icon(Icons.info_outline),
              color: RobotColors.primaryIcon,
              iconSize: 64,
            ),
          ),
        )
      ],
    );
  }
}
