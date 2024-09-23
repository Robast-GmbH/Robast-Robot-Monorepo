import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/auth_view.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';
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
  Timer? finishedTimer;
  bool waitingForFinish = false;
  int finishTimerClockIndex = 0;

  @override
  void initState() {
    super.initState();
    isDisinfected = !widget.requireDesinfection;
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

  @override
  void dispose() {
    finishedTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: Selector<ModuleProvider, List<Submodule>>(
            selector: (_, provider) => provider.submodules,
            builder: (context, modules, child) {
              final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
              if (modules.isEmpty || modules.every((module) => module.moduleProcess.status == ModuleProcessStatus.idle)) {
                return const Center(child: CircularProgressIndicator());
              }
              final moduleInProcess = moduleProvider.submodules.firstWhere(
                (element) => element.moduleProcess.status != ModuleProcessStatus.idle,
              );
              final position = moduleProvider.modules.indexWhere((submodules) => submodules.contains(moduleInProcess)) + 1;
              if (isDisinfected && moduleInProcess.moduleProcess.status == ModuleProcessStatus.waitingForOpening && !openingTriggered) {
                openingTriggered = true;
                WidgetsBinding.instance.addPostFrameCallback((_) {
                  moduleProvider.openSubmodule(moduleInProcess);
                });
                return const Center(child: CircularProgressIndicator());
              }

              if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.auth) {
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

              if (moduleInProcess.moduleProcess.status != ModuleProcessStatus.closed) {
                return Row(
                  children: [
                    const Expanded(child: SizedBox()),
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.symmetric(vertical: 64),
                        child: Stack(
                          children: [
                            if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.opening) ...[
                              HintView(
                                text: moduleInProcess.variant == SubmoduleVariant.electric || moduleInProcess.variant == SubmoduleVariant.partial
                                    ? 'Gewählte Schublade öffnet sich'
                                    : 'Bitte gewählte Schublade öffnen',
                                moduleLabel: 'Modul $position',
                              ),
                            ],
                            if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.open) ...[
                              GestureDetector(
                                onTap: () {
                                  if (moduleInProcess.variant == SubmoduleVariant.electric || moduleInProcess.variant == SubmoduleVariant.partial) {
                                    moduleProvider.closeSubmodule(moduleInProcess);
                                  }
                                },
                                child: HintView(
                                  text:
                                      '${moduleInProcess.moduleProcess.itemsByChangeToString()}${moduleInProcess.variant == SubmoduleVariant.electric || moduleInProcess.variant == SubmoduleVariant.partial ? ' Zum Schließen tippen.' : ''}',
                                  moduleLabel: 'Modul $position',
                                ),
                              ),
                            ],
                            if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.closing) ...[
                              HintView(
                                text: 'Schublade schließt sich, bitte warten',
                                moduleLabel: 'Modul $position',
                              ),
                            ],
                          ],
                        ),
                      ),
                    ),
                    const Expanded(child: SizedBox()),
                  ],
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
                return Row(
                  children: [
                    const Expanded(child: SizedBox()),
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.symmetric(vertical: 192),
                        child: Column(
                          children: [
                            Expanded(
                              flex: 2,
                              child: CustomButtonView(
                                text: 'Fertigstellen',
                                content: Text(
                                  '(automatisch in ${5 - finishTimerClockIndex} Sekunden)',
                                  style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                ),
                                onPressed: () => onFinish(moduleInProcess),
                              ),
                            ),
                            const SizedBox(
                              height: 8,
                            ),
                            Expanded(
                              child: CustomButtonView(
                                text: 'Erneut öffnen',
                                onPressed: () async {
                                  await moduleProvider.openSubmodule(moduleInProcess);
                                  finishedTimer?.cancel();
                                  waitingForFinish = false;
                                  finishTimerClockIndex = 0;
                                },
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    const Expanded(child: SizedBox()),
                  ],
                );
              } else {
                return const SizedBox();
              }
            },
          ),
        ),
      ],
    );
  }
}
