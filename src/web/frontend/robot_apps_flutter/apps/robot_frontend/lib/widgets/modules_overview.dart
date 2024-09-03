import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/module_filling_page.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class ModulesOverview extends StatelessWidget {
  const ModulesOverview({
    required this.submodules,
    this.displayReservations = false,
    super.key,
  });

  final List<Submodule> submodules;
  final bool displayReservations;

  @override
  Widget build(BuildContext context) {
    final moduleIDsBySubmodules = <int, List<Submodule>>{};
    for (final submodule in submodules) {
      final moduleID = submodule.address.moduleID;
      if (!moduleIDsBySubmodules.containsKey(moduleID)) {
        moduleIDsBySubmodules[moduleID] = [];
      }
      moduleIDsBySubmodules[moduleID]!.add(submodule);
    }
    return Padding(
      padding: const EdgeInsets.all(32),
      child: ListView(
        children: moduleIDsBySubmodules.keys
            .map(
              (moduleID) => Padding(
                padding: const EdgeInsets.only(bottom: 8),
                child: RoundedContainer(
                  child: ExpansionTile(
                    iconColor: RobotColors.primaryIcon,
                    collapsedIconColor: RobotColors.primaryIcon,
                    initiallyExpanded: true,
                    shape: const Border(),
                    title: Text(
                      'Modul ${moduleIDsBySubmodules[moduleID]!.first.position}',
                      style: const TextStyle(fontSize: 32),
                    ),
                    children: moduleIDsBySubmodules[moduleID]!.map((submodule) {
                      final isReserved = submodule.isReserved();
                      return InkWell(
                        onTap: () {
                          Navigator.push(
                            context,
                            MaterialPageRoute<ModuleFillingPage>(
                              builder: (context) => ModuleFillingPage(
                                submodule: submodule,
                              ),
                            ),
                          );
                        },
                        child: Padding(
                          padding: const EdgeInsets.all(8),
                          child: RoundedContainer(
                            child: Row(
                              mainAxisAlignment: MainAxisAlignment.spaceBetween,
                              children: [
                                Padding(
                                  padding: const EdgeInsets.all(16),
                                  child: Row(
                                    children: [
                                      Text(
                                        'Submodul ${submodule.address.submoduleID}',
                                        style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                      ),
                                      if (displayReservations) ...[
                                        const SizedBox(width: 8),
                                        if (submodule.reservedForTask.isNotEmpty) ...[
                                          const Text('für Auftrag reserviert', style: TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
                                        ] else if (submodule.reservedForIds.isNotEmpty) ...[
                                          const Text('für Nutzer reserviert', style: TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
                                        ] else if (submodule.reservedForGroups.isNotEmpty) ...[
                                          const Text('für Nutzergruppen reserviert', style: TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
                                        ] else ...[
                                          const Text('frei', style: TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
                                        ],
                                      ],
                                    ],
                                  ),
                                ),
                                Row(
                                  children: [
                                    Padding(
                                      padding: const EdgeInsets.all(16),
                                      child: Text(
                                        submodule.itemsByCount.entries
                                            .map((e) => ' ${e.key}: ${e.value}')
                                            .fold('', (previousValue, element) => previousValue + element),
                                        style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                      ),
                                    ),
                                    if (displayReservations) ...[
                                      TextButton(
                                        onPressed: () async {
                                          final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
                                          if (submodule.reservedForTask.isNotEmpty) {
                                            final cancelFreeing = await showDialog<bool>(
                                              context: context,
                                              builder: (context) {
                                                return AlertDialog(
                                                  title: const Text('Auftrag freigeben?'),
                                                  content: const Text('Freigeben des Submoduls beendet den assoziierten Auftrag.'),
                                                  actions: [
                                                    TextButton(
                                                      onPressed: () {
                                                        Navigator.pop(context, false);
                                                      },
                                                      child: const Text('Bestätigen'),
                                                    ),
                                                    TextButton(
                                                      onPressed: () {
                                                        Navigator.pop(context, true);
                                                      },
                                                      child: const Text('Abbrechen'),
                                                    ),
                                                  ],
                                                );
                                              },
                                            );
                                            if (cancelFreeing ?? true) {
                                              return;
                                            }
                                          }
                                          if (isReserved) {
                                            await moduleProvider.freeSubmodule(submoduleAddress: submodule.address);
                                          } else {
                                            await moduleProvider.reserveSubmodule(submoduleAddress: submodule.address, userGroups: ['STAFF']);
                                          }
                                          await moduleProvider.fetchSubmodules();
                                        },
                                        child: isReserved
                                            ? const Text('Freigeben', style: TextStyle(fontSize: 24, color: RobotColors.primaryText))
                                            : const Text('Reservieren', style: TextStyle(fontSize: 24, color: RobotColors.primaryText)),
                                      ),
                                      const SizedBox(
                                        width: 16,
                                      ),
                                    ],
                                  ],
                                ),
                              ],
                            ),
                          ),
                        ),
                      );
                    }).toList(),
                  ),
                ),
              ),
            )
            .toList(),
      ),
    );
  }
}
