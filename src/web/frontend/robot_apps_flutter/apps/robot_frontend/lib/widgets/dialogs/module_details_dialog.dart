import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/module_pages/module_filling_page.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class ModuleDetailsDialog extends StatelessWidget {
  const ModuleDetailsDialog({required this.moduleID, super.key});

  final int moduleID;
  @override
  Widget build(BuildContext context) {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    final modulePosition = moduleProvider.modules.indexWhere((module) => module.any((submodule) => submodule.address.moduleID == moduleID)) + 1;
    return AlertDialog(
      backgroundColor: RobotColors.primaryBackground,
      title: Text(
        'Modul $modulePosition',
        style: const TextStyle(fontSize: 40, color: RobotColors.primaryText),
      ),
      content: ConstrainedBox(
        constraints: const BoxConstraints(minWidth: 600),
        child: FractionallySizedBox(
          heightFactor: 0.6,
          child: SingleChildScrollView(
            child: Selector<ModuleProvider, List<Submodule>>(
                selector: (context, provider) => provider.submodules,
                builder: (context, submodules, child) {
                  submodules = submodules.where((submodule) => moduleID == submodule.address.moduleID).toList();
                  return Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      for (final submodule in submodules)
                        Padding(
                          padding: const EdgeInsets.only(bottom: 16),
                          child: RoundedContainer(
                            child: Padding(
                              padding: const EdgeInsets.all(16),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Text(
                                    'Submodul ${submodule.address.submoduleID}',
                                    style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
                                  ),
                                  const SizedBox(height: 4),
                                  Text(
                                    'Status: ${submodule.isReserved() ? 'reserviert' : 'frei'}',
                                    style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                  ),
                                  Text(
                                    'Größe: ${submodule.variant == SubmoduleVariant.partial ? '1/8' : Submodule.sizesByDisplayName[submodule.size]!}',
                                    style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                  ),
                                  Text(
                                    'Variante: ${submodule.variant.toString().split('.').last}',
                                    style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                  ),
                                  const SizedBox(
                                    height: 4,
                                  ),
                                  RoundedContainer(
                                      child: Padding(
                                    padding: const EdgeInsets.all(16),
                                    child: Column(
                                      crossAxisAlignment: CrossAxisAlignment.stretch,
                                      children: [
                                        const Text(
                                          'Inhalte',
                                          style: TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                        ),
                                        if (submodule.itemsByCount.isNotEmpty)
                                          ...submodule.itemsByCount.entries.map((entry) => Text(
                                                '${entry.key}: ${entry.value}',
                                                style: const TextStyle(color: RobotColors.secondaryText, fontSize: 24),
                                              ))
                                        else
                                          const Text(
                                            'leer',
                                            style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
                                          ),
                                        Row(
                                          mainAxisAlignment: MainAxisAlignment.end,
                                          children: [
                                            TextButton(
                                              onPressed: submodule.isReserved()
                                                  ? null
                                                  : () async {
                                                      await Navigator.push(
                                                          context,
                                                          MaterialPageRoute<ModuleFillingPage>(
                                                            builder: (context) => ModuleFillingPage(
                                                              submodule: submodule,
                                                            ),
                                                          ));
                                                    },
                                              child: Row(
                                                children: [
                                                  Text(
                                                    'Verwalten',
                                                    style: TextStyle(
                                                        color: submodule.isReserved() ? RobotColors.disabled : RobotColors.secondaryText, fontSize: 24),
                                                  ),
                                                  const SizedBox(width: 8),
                                                  Icon(Icons.arrow_forward, color: submodule.isReserved() ? RobotColors.disabled : RobotColors.secondaryText),
                                                ],
                                              ),
                                            )
                                          ],
                                        ),
                                      ],
                                    ),
                                  ))
                                ],
                              ),
                            ),
                          ),
                        ),
                    ],
                  );
                }),
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: () {
            Navigator.pop(context);
          },
          child: const Text(
            'Schließen',
            style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
          ),
        ),
      ],
    );
  }
}
