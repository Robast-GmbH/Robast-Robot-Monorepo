import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/drawer_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
          child: Selector<RobotProvider, ModuleProcess?>(
            selector: (_, provider) => provider.moduleProcess,
            builder: (context, moduleProcess, child) {
              final modules = Provider.of<RobotProvider>(context, listen: false).modules;
              if (modules == null) {
                return const Center(child: CircularProgressIndicator());
              }
              if (moduleProcess?.state != ModuleProcessState.closed && moduleProcess?.state != ModuleProcessState.finished) {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 64),
                  child: Stack(
                    children: [
                      Column(
                        children: Provider.of<RobotProvider>(context, listen: false).modules?.map((module) {
                              return DrawerView(
                                module: module,
                                label: moduleProcess?.moduleID == module.moduleID ? moduleProcess?.payload : '',
                                isAnyDrawerOpen: moduleProcess?.moduleID != module.moduleID,
                                isEnabled: moduleProcess?.moduleID == module.moduleID,
                                onOpening: () {
                                  Provider.of<RobotProvider>(context, listen: false).openDrawer(module);
                                },
                              );
                            }).toList() ??
                            [],
                      ),
                      if (moduleProcess?.state == ModuleProcessState.opening) ...[
                        HintView(
                          text: modules[moduleProcess!.moduleID].type == ModuleType.electric_drawer
                              ? 'Gewählte Schublade öffnet sich'
                              : 'Bitte gewählte Schublade öffnen',
                          moduleLabel: moduleProcess.payload,
                        ),
                      ],
                      if (moduleProcess?.state == ModuleProcessState.closing) ...[
                        HintView(
                          text: 'Schublade schließt sich, bitte warten',
                          moduleLabel: moduleProcess!.payload,
                        ),
                      ],
                    ],
                  ),
                );
              }
              if (moduleProcess?.state == ModuleProcessState.closed) {
                return GestureDetector(
                  onTap: () {
                    final module = Provider.of<RobotProvider>(context, listen: false).modules?.firstWhere(
                          (module) => module.moduleID == moduleProcess?.moduleID,
                        );
                    switch (moduleProcess?.state) {
                      case ModuleProcessState.waitingForOpeningCommand:
                        Provider.of<RobotProvider>(context, listen: false).openDrawer(module!);
                      case ModuleProcessState.open:
                        Provider.of<RobotProvider>(context, listen: false).closeDrawer(module!);
                      case ModuleProcessState.closed:
                        break;
                      default:
                        break;
                    }
                  },
                  child: Padding(
                    padding: const EdgeInsets.symmetric(vertical: 196),
                    child: Column(
                      children: [
                        Expanded(
                          flex: 2,
                          child: CustomButtonView(text: 'Finish', onPressed: () async {}),
                        ),
                        const SizedBox(
                          height: 8,
                        ),
                        Expanded(
                          child: CustomButtonView(
                            text: 'Reopen',
                            onPressed: () {
                              final module = Provider.of<RobotProvider>(context, listen: false).modules?.firstWhere(
                                    (module) => module.moduleID == moduleProcess?.moduleID,
                                  );
                              Provider.of<RobotProvider>(context, listen: false).openDrawer(module!);
                            },
                          ),
                        ),
                      ],
                    ),
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
