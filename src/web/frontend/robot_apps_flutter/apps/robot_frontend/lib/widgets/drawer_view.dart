import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';

class DrawerView extends StatelessWidget {
  const DrawerView({
    required this.module,
    required this.isAnyDrawerOpen,
    required this.onOpening,
    super.key,
    this.label,
    this.isEnabled = true,
  });

  final RobotDrawer module;
  final bool isAnyDrawerOpen;
  final VoidCallback onOpening;
  final bool isEnabled;
  final String? label;

  @override
  Widget build(BuildContext context) {
    final moduleProcess = module.moduleProcess;
    final isOpen = moduleProcess.status == ModuleProcessStatus.open;
    return Expanded(
      flex: module.size,
      child: GestureDetector(
        onTap: () {
          if (!isEnabled) return;
          if (isOpen && module.variant == DrawerVariant.electric) {
            Provider.of<ModulesProvider>(context, listen: false).closeDrawer(module);
          } else if (moduleProcess.status == ModuleProcessStatus.waitingForOpeningCommand || moduleProcess.status == ModuleProcessStatus.closed) {
            Provider.of<ModulesProvider>(context, listen: false).openDrawer(module);
            onOpening();
          }
        },
        child: Column(
          children: [
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(4),
                child: Opacity(
                  opacity: isAnyDrawerOpen && !isOpen ? 0.2 : 1.0,
                  child: Container(
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.bottomCenter,
                        end: Alignment.topCenter,
                        colors: isOpen
                            ? [
                                const Color(0xCCBBFF33),
                                const Color(0x7FA8E52D),
                              ]
                            : [
                                Colors.white.withOpacity(0.5),
                                Colors.white.withOpacity(0.3),
                              ],
                      ),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: SizedBox.expand(
                      child: Align(
                        child: Text(
                          label ?? '',
                          textAlign: TextAlign.center,
                          style: const TextStyle(
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
              ),
            ),
          ],
        ),
      ),
    );
  }
}
