import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
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
        Expanded(child: SizedBox()),
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
                  padding: const EdgeInsets.symmetric(vertical: 64.0),
                  child: Stack(
                    children: [
                      Column(
                        children: Provider.of<RobotProvider>(context, listen: false).modules?.map((module) {
                              return DrawerView(
                                module: module,
                                label: moduleProcess?.moduleID == module.moduleID ? moduleProcess?.payload : "",
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
                        break;
                      case ModuleProcessState.open:
                        Provider.of<RobotProvider>(context, listen: false).closeDrawer(module!);
                        break;
                      case ModuleProcessState.closed:
                        Provider.of<RobotProvider>(context, listen: false).finishModuleProcess();
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
                          child: buildButtonView("Finish", () async {
                            await Provider.of<RobotProvider>(context, listen: false).finishModuleProcess();
                          }),
                        ),
                        SizedBox(
                          height: 8,
                        ),
                        Expanded(
                          child: buildButtonView("Reopen", () {
                            final module = Provider.of<RobotProvider>(context, listen: false).modules?.firstWhere(
                                  (module) => module.moduleID == moduleProcess?.moduleID,
                                );
                            Provider.of<RobotProvider>(context, listen: false).openDrawer(module!);
                          }),
                        ),
                      ],
                    ),
                  ),
                );
              } else {
                return SizedBox();
              }
            },
          ),
        ),
        Expanded(child: SizedBox()),
      ],
    );
  }

  Widget buildButtonView(String text, VoidCallback onPressed) {
    return GestureDetector(
      onTap: onPressed,
      child: Padding(
        padding: const EdgeInsets.all(4),
        child: Container(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.bottomCenter,
              end: Alignment.topCenter,
              colors: [
                Colors.white.withOpacity(0.5),
                Colors.white.withOpacity(0.3),
              ],
            ),
            borderRadius: BorderRadius.circular(12),
          ),
          child: SizedBox.expand(
            child: Align(
              alignment: Alignment.center,
              child: Text(
                text,
                textAlign: TextAlign.center,
                style: TextStyle(
                  height: 0,
                  color: Colors.white,
                  fontSize: 40,
                  fontWeight: FontWeight.w400,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
