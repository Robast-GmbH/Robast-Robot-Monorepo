import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/widgets/module_details_dialog.dart';
import 'package:web_frontend/widgets/rounded_button.dart';

class ModulesOverview extends StatelessWidget {
  const ModulesOverview({required this.robotName, super.key});

  final String robotName;

  String getModuleStatus(List<Submodule> submodules) {
    var reservedCount = 0;
    var isLoaded = false;
    for (final submodule in submodules) {
      if (submodule.isReserved()) {
        reservedCount++;
      }
      if (submodule.itemsByCount.isNotEmpty) {
        isLoaded = true;
      }
    }
    var moduleStatus = isLoaded ? 'beladen, ' : 'leer, ';
    if (reservedCount == submodules.length) {
      moduleStatus += 'reserviert';
    } else if (reservedCount > 0) {
      moduleStatus += 'teilweise frei';
    } else {
      moduleStatus += 'frei';
    }
    return moduleStatus;
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: Selector<FleetProvider, Map<String, List<Submodule>>>(
        selector: (context, provider) => provider.modules,
        builder: (context, robotsBySubmodules, child) {
          if (!robotsBySubmodules.containsKey('rb_theron')) {}
          final submodules = robotsBySubmodules['rb_theron']!;
          final moduleIDsBySubmodules = <int, List<Submodule>>{};
          for (final submodule in submodules) {
            final moduleID = submodule.address.moduleID;
            if (!moduleIDsBySubmodules.containsKey(moduleID)) {
              moduleIDsBySubmodules[moduleID] = [];
            }
            moduleIDsBySubmodules[moduleID]!.add(submodule);
          }

          return Column(
            children: [
              ...List.generate(moduleIDsBySubmodules.keys.length, (index) {
                final moduleID = moduleIDsBySubmodules.keys.elementAt(index);
                return SizedBox(
                  height: moduleIDsBySubmodules[moduleID]!.first.size * 100,
                  width: double.infinity,
                  child: Column(
                    children: [
                      const SizedBox(height: 16),
                      Expanded(
                        child: RoundedButton(
                          onPressed: () {
                            showDialog<ModuleDetailsDialog>(
                              context: context,
                              builder: (context) => ModuleDetailsDialog(
                                moduleID: moduleID,
                                position: index + 1,
                                submodules: moduleIDsBySubmodules[moduleID]!,
                              ),
                            );
                          },
                          color: Colors.white.withOpacity(0.2),
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Text(
                                'Modul ${index + 1}',
                                style: const TextStyle(
                                  fontSize: 24,
                                  color: WebColors.secondaryText,
                                ),
                              ),
                              Text(
                                getModuleStatus(moduleIDsBySubmodules[moduleID]!),
                                style: const TextStyle(fontSize: 22, color: WebColors.secondaryText),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ],
                  ),
                );
              }),
              const SizedBox(height: 16),
            ],
          );
        },
      ),
    );
  }
}
