import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/dialogs/module_details_dialog.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';

class MenuModulesOverview extends StatelessWidget {
  const MenuModulesOverview({super.key});

  String getModuleStatus(List<Submodule> submodules) {
    var reservedCount = 0;
    bool isLoaded = false;
    for (final submodule in submodules) {
      if (submodule.isReserved()) {
        reservedCount++;
      }
      if (submodule.itemsByCount.isNotEmpty) {
        isLoaded = true;
      }
    }
    String moduleStatus = isLoaded ? 'beladen, ' : 'leer, ';
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
    return CustomButtonView(
      text: 'Module',
      onPressed: () {},
      content: Selector<ModuleProvider, List<Submodule>>(
        selector: (context, provider) => provider.submodules,
        builder: (context, submodules, child) {
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
              ...moduleIDsBySubmodules.keys.map(
                (moduleID) => Expanded(
                  flex: moduleIDsBySubmodules[moduleID]!.first.size,
                  child: Column(
                    children: [
                      const SizedBox(height: 16),
                      Expanded(
                        child: RoundedButton(
                          onPressed: () {
                            showDialog(
                              context: context,
                              builder: (context) => ModuleDetailsDialog(
                                moduleID: moduleID,
                              ),
                            );
                          },
                          color: Colors.black.withOpacity(0.1),
                          child: Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Text(
                                'Modul $moduleID',
                                style: const TextStyle(
                                  fontSize: 32,
                                  color: RobotColors.secondaryText,
                                ),
                              ),
                              Text(
                                getModuleStatus(moduleIDsBySubmodules[moduleID]!),
                                style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }
}
