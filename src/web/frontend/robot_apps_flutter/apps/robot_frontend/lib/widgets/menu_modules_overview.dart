import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/module_pages/module_process_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';

class MenuModulesOverview extends StatelessWidget {
  const MenuModulesOverview({this.isDemo = false, super.key});

  final bool isDemo;

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
    if (isDemo) {
      return isLoaded ? 'beladen' : 'leer';
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
                          if (index == 1) return;
                          final moduleProvider = Provider.of<ModuleProvider>(context, listen: false)..isInSubmoduleProcess = true;
                          await moduleProvider.startSubmoduleProcess(
                            submoduleAddress: modules[index].first.address,
                            processName: 'fill',
                            itemsByChange: {'Süßigkeiten': -2},
                          );
                          if (context.mounted) {
                            await Navigator.push(context, MaterialPageRoute<ModuleProcessPage>(builder: (context) => const ModuleProcessPage()));
                          }
                          moduleProvider.isInSubmoduleProcess = false;

                          // showDialog(
                          //   context: context,
                          //   builder: (context) => ModuleDetailsDialog(
                          //     moduleID: modules[index].first.address.moduleID,
                          //   ),
                          // );
                        },
                        color: index == 1 ? Colors.black.withOpacity(0.05) : null,
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Text(
                              'Modul ${index + 1}',
                              style: TextStyle(
                                fontSize: 32,
                                color: index == 1 ? RobotColors.tertiaryText : RobotColors.secondaryText,
                              ),
                            ),
                            Text(
                              index == 1 ? 'reserviert' : getModuleStatus(modules[index]),
                              style: TextStyle(
                                fontSize: 24,
                                color: index == 1 ? RobotColors.tertiaryText : RobotColors.secondaryText,
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
