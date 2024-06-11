import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/drawer_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class StoppedView extends StatefulWidget {
  const StoppedView({super.key, required this.onContinue});
  final Future<void> Function() onContinue;

  @override
  State<StoppedView> createState() => _StoppedViewState();
}

class _StoppedViewState extends State<StoppedView> {
  final isOpening = [false, false, false, false, false, false];
  final isClosing = [false, false, false, false, false, false];

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTap: () async {
        final isAnyDrawerOpen = Provider.of<RobotProvider>(context, listen: false).modules?.any((module) => module.isOpen) ?? false;
        if (!isAnyDrawerOpen && !isOpening.any((e) => e)) await widget.onContinue();
      },
      child: Row(children: [
        const Expanded(
          child: SizedBox(),
        ),
        Expanded(
          child: Selector<RobotProvider, List<DrawerModule>?>(
            selector: (_, provider) => provider.modules,
            builder: (context, modules, child) {
              if (modules == null) {
                return const Center(child: Text("Loading..."));
              }
              final isAnyDrawerOpen = modules.any((module) => module.isOpen);
              if (isAnyDrawerOpen) {
                for (int i = 0; i < modules.length; i++) {
                  isOpening[i] = false;
                }
              } else {
                for (int i = 0; i < modules.length; i++) {
                  isClosing[i] = false;
                }
              }
              return Column(
                children: [
                  Expanded(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(vertical: 64),
                      child: Container(
                        decoration: BoxDecoration(
                          color: Colors.white.withOpacity(0.25),
                          borderRadius: BorderRadius.circular(18),
                        ),
                        padding: EdgeInsets.all(4),
                        child: Stack(
                          children: [
                            Column(
                              children: modules
                                  .map((module) => DrawerView(
                                        module: module,
                                        isAnyDrawerOpen: isAnyDrawerOpen,
                                        onOpening: () {
                                          setState(() {
                                            isOpening[modules.indexOf(module)] = true;
                                          });
                                        },
                                      ))
                                  .toList(),
                            ),
                            if (isOpening.any((e) => e)) ...[
                              HintView(
                                text: modules[isOpening.indexOf(true)].type == ModuleType.electric_drawer
                                    ? 'Gewählte Schublade öffnet sich'
                                    : 'Bitte gewählte Schublade öffnen',
                                moduleLabel: modules[isOpening.indexOf(true)].label,
                              ),
                            ],
                            if (isAnyDrawerOpen) ...[
                              GestureDetector(
                                onTap: () {
                                  final module = modules.firstWhere((module) => module.isOpen);
                                  if (module.type == ModuleType.electric_drawer) {
                                    Provider.of<RobotProvider>(context, listen: false).closeDrawer(module);
                                    isClosing[modules.indexOf(module)] = true;
                                  }
                                },
                                child: Container(
                                  decoration: BoxDecoration(
                                    gradient: LinearGradient(begin: Alignment.bottomCenter, end: Alignment.topCenter, colors: [
                                      Color.fromARGB(255, 135, 184, 37),
                                      Color.fromARGB(255, 157, 212, 47),
                                    ]),
                                    borderRadius: BorderRadius.circular(12),
                                  ),
                                  margin: EdgeInsets.all(4),
                                  child: SizedBox(
                                    width: double.infinity,
                                    height: double.infinity,
                                    child: Column(
                                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                                      children: [
                                        Padding(
                                          padding: const EdgeInsets.only(top: 8.0),
                                          child: Text(
                                            modules.firstWhere((module) => module.isOpen).label,
                                            style: TextStyle(
                                              height: 0,
                                              color: Colors.white,
                                              fontSize: 40,
                                              fontWeight: FontWeight.w400,
                                            ),
                                          ),
                                        ),
                                        Padding(
                                          padding: const EdgeInsets.symmetric(horizontal: 48),
                                          child: Text(
                                            modules.firstWhere((module) => module.isOpen).type == ModuleType.electric_drawer
                                                ? 'Zum Schließen der Schublade bitte hier berühren'
                                                : 'Zum Fortfahren Schublade bitte wieder schließen',
                                            textAlign: TextAlign.center,
                                            style: TextStyle(
                                              color: Colors.white,
                                              fontSize: 40,
                                              fontWeight: FontWeight.w500,
                                            ),
                                          ),
                                        ),
                                        Padding(
                                          padding: const EdgeInsets.only(bottom: 64),
                                          child: Icon(
                                              modules.firstWhere((module) => module.isOpen).type == ModuleType.manual_drawer
                                                  ? Icons.arrow_downward
                                                  : Icons.touch_app,
                                              size: 100,
                                              color: Colors.white),
                                        ),
                                      ],
                                    ),
                                  ),
                                ),
                              )
                            ],
                            if (isClosing.any((e) => e)) ...[
                              HintView(
                                text: 'Schublade schließt sich, bitte warten',
                                moduleLabel: modules[isClosing.indexOf(true)].label,
                              ),
                            ],
                          ],
                        ),
                      ),
                    ),
                  ),
                  Padding(
                    padding: const EdgeInsets.only(bottom: 48),
                    child: Text(
                      'Fortfahren',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        letterSpacing: 1.5,
                        color: isAnyDrawerOpen || isOpening.any((e) => e) ? Colors.transparent : Colors.white,
                        fontSize: 64,
                        fontWeight: FontWeight.w300,
                      ),
                    ),
                  ),
                ],
              );
            },
          ),
        ),
        Expanded(child: const SizedBox()),
      ]),
    );
  }
}
