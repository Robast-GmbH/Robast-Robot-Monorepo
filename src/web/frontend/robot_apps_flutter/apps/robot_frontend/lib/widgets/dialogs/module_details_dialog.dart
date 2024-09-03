import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class ModuleDetailsDialog extends StatelessWidget {
  const ModuleDetailsDialog({required this.moduleID, required this.submodules, super.key});

  final int moduleID;
  final List<Submodule> submodules;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: RobotColors.primaryBackground,
      title: Text(
        'Modul $moduleID',
        style: const TextStyle(fontSize: 40, color: RobotColors.primaryText),
      ),
      content: ConstrainedBox(
        constraints: const BoxConstraints(minWidth: 600),
        child: FractionallySizedBox(
          heightFactor: 0.6,
          child: SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                for (final submodule in submodules)
                  RoundedContainer(
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
                            'Größe: ${SubmoduleTypes.values[submodule.size - 1].toString().split('.').last}',
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
                                  ...submodule.itemsByCount.entries
                                      .map((entry) => Text(
                                            '${entry.key}: ${entry.value}',
                                            style: const TextStyle(color: RobotColors.secondaryText, fontSize: 24),
                                          ))
                                      
                                else
                                  const Text(
                                    'leer',
                                    style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
                                  ),
                              ],
                            ),
                          ))
                        ],
                      ),
                    ),
                  ),
              ],
            ),
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
