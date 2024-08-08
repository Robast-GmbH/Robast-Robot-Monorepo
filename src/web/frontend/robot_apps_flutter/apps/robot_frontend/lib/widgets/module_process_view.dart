import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/auth_view.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';
import 'package:robot_frontend/widgets/drawer_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';

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
  Timer? finishedTimer;
  bool waitingForFinish = false;
  int finishTimerClockIndex = 0;

  @override
  void initState() {
    super.initState();
    isDisinfected = !widget.requireDesinfection;
  }

  Future<void> onFinish(RobotDrawer moduleInProcess) async {
    if (!mounted) {
      return;
    }
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    await moduleProvider.finishModuleProcess(moduleInProcess);
    await moduleProvider.fetchModules();
    finishedTimer?.cancel();
    if (mounted) {
      Navigator.pop(context);
    }
  }

  @override
  void dispose() {
    finishedTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        const Expanded(child: SizedBox()),
        Expanded(
          child: Selector<ModuleProvider, List<RobotDrawer>>(
            selector: (_, provider) => provider.modules,
            builder: (context, modules, child) {
              if (modules.isEmpty || modules.every((module) => module.moduleProcess.status == ModuleProcessStatus.idle)) {
                return const Center(child: CircularProgressIndicator());
              }
              final moduleInProcess = Provider.of<ModuleProvider>(context).modules.firstWhere(
                    (element) => element.moduleProcess.status != ModuleProcessStatus.idle,
                  );
              if (isDisinfected && moduleInProcess.moduleProcess.status == ModuleProcessStatus.waitingForOpening && !openingTriggered) {
                openingTriggered = true;
                WidgetsBinding.instance.addPostFrameCallback((_) {
                  Provider.of<ModuleProvider>(context, listen: false).openDrawer(moduleInProcess);
                });
                return const Center(child: CircularProgressIndicator());
              }

              if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.auth) {
                return AuthView(
                  requiredUserIDs: moduleInProcess.reservedForIds,
                  requiredUserGroups: moduleInProcess.reservedForGroups,
                  onAuthCompleted: (wasAuthSuccessful) {
                    if (wasAuthSuccessful) {
                      Provider.of<ModuleProvider>(context, listen: false).fetchModules();
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

              if (moduleInProcess.moduleProcess.status != ModuleProcessStatus.closed) {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 64),
                  child: Stack(
                    children: [
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.opening) ...[
                        HintView(
                          text: modules[moduleInProcess.moduleID - 1].variant == DrawerVariant.electric
                              ? 'Gewählte Schublade öffnet sich'
                              : 'Bitte gewählte Schublade öffnen',
                          moduleLabel: 'Modul ${moduleInProcess.moduleID}',
                        ),
                      ],
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.open) ...[
                        GestureDetector(
                          onTap: () {
                            if (moduleInProcess.variant == DrawerVariant.electric) {
                              Provider.of<ModuleProvider>(context, listen: false).closeDrawer(moduleInProcess);
                            }
                          },
                          child: HintView(
                            text:
                                '${moduleInProcess.moduleProcess.itemsByChangeToString()}${moduleInProcess.variant == DrawerVariant.electric ? ' Zum Schließen tippen.' : ''}',
                            moduleLabel: 'Modul ${moduleInProcess.moduleID}',
                          ),
                        ),
                      ],
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.closing) ...[
                        HintView(
                          text: 'Schublade schließt sich, bitte warten',
                          moduleLabel: 'Module ${moduleInProcess.moduleID}',
                        ),
                      ],
                    ],
                  ),
                );
              }
              if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.closed) {
                if (!waitingForFinish) {
                  waitingForFinish = true;
                  finishedTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
                    finishTimerClockIndex++;
                    setState(() {});
                    if (finishTimerClockIndex == 5) {
                      onFinish(moduleInProcess);
                    }
                  });
                }
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 196),
                  child: Column(
                    children: [
                      Expanded(
                        flex: 2,
                        child: CustomButtonView(
                          text: 'Finish  ${5 - finishTimerClockIndex}',
                          onPressed: () => onFinish(moduleInProcess),
                        ),
                      ),
                      const SizedBox(
                        height: 8,
                      ),
                      Expanded(
                        child: CustomButtonView(
                          text: 'Reopen',
                          onPressed: () async {
                            await Provider.of<ModuleProvider>(context, listen: false).openDrawer(moduleInProcess);
                            finishedTimer?.cancel();
                            waitingForFinish = false;
                            finishTimerClockIndex = 0;
                          },
                        ),
                      ),
                    ],
                  ),
                );
              } else {
                return const SizedBox();
              }
            },
          ),
        ),
        const Expanded(child: SizedBox()),
      ],
    );
  }
}
