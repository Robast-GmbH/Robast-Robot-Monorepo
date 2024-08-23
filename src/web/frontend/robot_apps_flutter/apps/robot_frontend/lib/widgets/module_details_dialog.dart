import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

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
        style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
      ),
      content: FractionallySizedBox(
        heightFactor: 0.4,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            for (final submodule in submodules)
              Card(
                color: Colors.black.withOpacity(0.2),
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Submodul ${submodule.address.submoduleID}',
                        style: const TextStyle(fontSize: 28, color: RobotColors.primaryText),
                      ),
                      const SizedBox(height: 8),
                      Row(
                        children: [
                          Text(
                            'Status: ${submodule.isReserved() ? 'reserviert' : 'frei'}',
                            style: const TextStyle(color: RobotColors.secondaryText),
                          ),
                          const SizedBox(width: 16),
                          Text(
                            'Typ: ${submodule.size}',
                            style: const TextStyle(color: RobotColors.secondaryText),
                          ),
                          const SizedBox(width: 16),
                          Text(
                            'Variante: ${submodule.variant.toString().split('.').last}',
                            style: const TextStyle(color: RobotColors.secondaryText),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ),
          ],
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
