import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class ModuleDetailsDialog extends StatelessWidget {
  const ModuleDetailsDialog({required this.moduleID, required this.position, required this.submodules, super.key});

  final int moduleID;
  final int position;
  final List<Submodule> submodules;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(
        'Modul $position',
        style: const TextStyle(fontSize: 24, color: WebColors.primaryText),
      ),
      content: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            for (final submodule in submodules)
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 8.0),
                child: RoundedContainer(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Submodul ${submodule.address.submoduleID}',
                          style: const TextStyle(fontSize: 20, color: WebColors.primaryText),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          'Status: ${submodule.isReserved() ? 'reserviert' : 'frei'}',
                          style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
                        ),
                        Text(
                          'Größe: ${submodule.variant == SubmoduleVariant.partial ? '1/8' : SubmoduleTypes.values[submodule.size - 1].toString().split('.').last}',
                          style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
                        ),
                        Text(
                          'Variante: ${submodule.variant.toString().split('.').last}',
                          style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
                        ),
                        const SizedBox(
                          height: 4,
                        ),
                        RoundedContainer(
                            color: Colors.white.withOpacity(0.2),
                            child: Padding(
                              padding: const EdgeInsets.all(16),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.stretch,
                                children: [
                                  const Text(
                                    'Inhalte',
                                    style: TextStyle(color: WebColors.secondaryText, fontSize: 18),
                                  ),
                                  if (submodule.itemsByCount.isNotEmpty)
                                    ...submodule.itemsByCount.entries.map((entry) => Text(
                                          '${entry.key}: ${entry.value}',
                                          style: const TextStyle(color: WebColors.secondaryText, fontSize: 16),
                                        ))
                                  else
                                    const Text(
                                      'leer',
                                      style: TextStyle(color: WebColors.secondaryText, fontSize: 16),
                                    ),
                                ],
                              ),
                            )),
                      ],
                    ),
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
          ),
        ),
      ],
    );
  }
}
