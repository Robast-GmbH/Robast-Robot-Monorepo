import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/drawer_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';

class ModuleProcessView extends StatefulWidget {
  const ModuleProcessView({super.key});

  @override
  State<ModuleProcessView> createState() => _ModuleProcessViewState();
}

class _ModuleProcessViewState extends State<ModuleProcessView> {
  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        const Expanded(child: SizedBox()),
        Expanded(
          child: Selector<ModulesProvider, List<RobotDrawer>>(
            selector: (_, provider) => provider.modules,
            builder: (context, modules, child) {
              if (modules.isEmpty || modules.every((module) => module.moduleProcess.status == ModuleProcessStatus.idle)) {
                return const Center(child: CircularProgressIndicator());
              }
              final moduleInProcess = Provider.of<ModulesProvider>(context).modules.firstWhere(
                    (element) => element.moduleProcess.status != ModuleProcessStatus.idle,
                  );
              if (moduleInProcess.moduleProcess.status != ModuleProcessStatus.closed) {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 64),
                  child: Stack(
                    children: [
                      Column(
                        children: modules.map((module) {
                          return DrawerView(
                            module: module,
                            isAnyDrawerOpen: moduleInProcess.moduleID != module.moduleID,
                            isEnabled: moduleInProcess.moduleID == module.moduleID,
                            onOpening: () {
                              Provider.of<ModulesProvider>(context, listen: false).openDrawer(module);
                            },
                            label: moduleInProcess == module ? 'Zum Öffnen tippen' : '',
                          );
                        }).toList(),
                      ),
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.opening) ...[
                        HintView(
                          text: modules[moduleInProcess.moduleID].variant == DrawerVariant.electric
                              ? 'Gewählte Schublade öffnet sich'
                              : 'Bitte gewählte Schublade öffnen',
                          moduleLabel: moduleInProcess.moduleProcess.payload.toString(),
                        ),
                      ],
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.open) ...[
                        GestureDetector(
                          onTap: () {
                            if (moduleInProcess.variant == DrawerVariant.electric) {
                              Provider.of<ModulesProvider>(context, listen: false).closeDrawer(moduleInProcess);
                            }
                          },
                          child: HintView(
                            text: moduleInProcess.moduleProcess.payloadToString(),
                            moduleLabel: moduleInProcess.moduleProcess.payload.toString(),
                          ),
                        ),
                      ],
                      if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.closing) ...[
                        HintView(
                          text: 'Schublade schließt sich, bitte warten',
                          moduleLabel: moduleInProcess.moduleProcess.payload.toString(),
                        ),
                      ],
                    ],
                  ),
                );
              }
              if (moduleInProcess.moduleProcess.status == ModuleProcessStatus.closed) {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 196),
                  child: Column(
                    children: [
                      Expanded(
                        flex: 2,
                        child: CustomButtonView(
                          text: 'Finish',
                          onPressed: () async {
                            Provider.of<ModulesProvider>(context, listen: false).finishModuleProcess(moduleInProcess);
                            Navigator.popUntil(context, (route) => route.isFirst);
                          },
                        ),
                      ),
                      const SizedBox(
                        height: 8,
                      ),
                      Expanded(
                        child: CustomButtonView(
                          text: 'Reopen',
                          onPressed: () {
                            Provider.of<ModulesProvider>(context, listen: false).openDrawer(moduleInProcess);
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
