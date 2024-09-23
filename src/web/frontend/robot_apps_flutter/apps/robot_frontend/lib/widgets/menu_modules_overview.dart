import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:robot_frontend/widgets/dialogs/module_details_dialog.dart';
import 'package:shared_data_models/shared_data_models.dart';

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
    String moduleStatus = isLoaded ? 'beladen' : 'leer';
    // if (reservedCount == submodules.length) {
    //   moduleStatus += 'reserviert';
    // } else if (reservedCount > 0) {
    //   moduleStatus += 'teilweise frei';
    // } else {
    //   moduleStatus += 'frei';
    // }
    return moduleStatus;
  }

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      text: 'Module',
      onPressed: () {},
      content: Selector<ModuleProvider, List<List<Submodule>>>(
        selector: (context, provider) => provider.modules,
        builder: (context, modules, child) {
          return Column(
            children: List.generate(
              modules.length,
              (index) => Expanded(
                flex: modules[index].first.size,
                child: Column(
                  children: [
                    const SizedBox(height: 16),
                    Expanded(
                      child: RoundedButton(
                        onPressed: () async {
                          showDialog(
                            context: context,
                            builder: (context) => ModuleDetailsDialog(
                              moduleID: modules[index].first.address.moduleID,
                            ),
                          );
                        },
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Text(
                              'Modul ${index + 1}',
                              style: const TextStyle(
                                fontSize: 32,
                                color: RobotColors.secondaryText,
                              ),
                            ),
                            Text(
                              getModuleStatus(modules[index]),
                              style: const TextStyle(
                                fontSize: 24,
                                color: RobotColors.secondaryText,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ).toList(),
          );
        },
      ),
    );
  }
}
