import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';

class DrawerView extends StatelessWidget {
  const DrawerView({
    required this.module, required this.isAnyDrawerOpen, required this.onOpening, super.key,
    this.label,
    this.isEnabled = true,
  });

  final DrawerModule module;
  final bool isAnyDrawerOpen;
  final VoidCallback onOpening;
  final bool isEnabled;
  final String? label;

  @override
  Widget build(BuildContext context) {
    return Expanded(
      flex: module.size,
      child: GestureDetector(
        onTap: () {
          if (!isEnabled) return;
          if (module.isOpen && module.type == ModuleType.electric_drawer) {
            Provider.of<RobotProvider>(context, listen: false).closeDrawer(module);
          } else if (!module.isOpen) {
            Provider.of<RobotProvider>(context, listen: false).openDrawer(module);
            onOpening();
          }
        },
        child: Column(
          children: [
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(4),
                child: Opacity(
                  opacity: isAnyDrawerOpen && !module.isOpen ? 0.2 : 1.0,
                  child: Container(
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.bottomCenter,
                        end: Alignment.topCenter,
                        colors: module.isOpen
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
                          label ?? module.label,
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
