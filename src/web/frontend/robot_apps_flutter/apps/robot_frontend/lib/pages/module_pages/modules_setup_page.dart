import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:shared_data_models/shared_data_models.dart';

class ModulesSetupPage extends StatefulWidget {
  const ModulesSetupPage({super.key});

  @override
  State<ModulesSetupPage> createState() => _ModulesSetupPageState();
}

class _ModulesSetupPageState extends State<ModulesSetupPage> {
  final List<bool> slotOccupancy = List.generate(8, (index) => false);
  final List<bool> selectedSlots = List.generate(8, (index) => false);
  final controller = TextEditingController();
  bool isElectric = false;
  bool isPartial = false;

  void updateSlots(List<Submodule> submodules) {
    slotOccupancy.fillRange(0, slotOccupancy.length, false);
    for (final submodule in submodules) {
      for (var i = 0; i < submodule.size; i++) {
        slotOccupancy[submodule.position + i - 1] = true;
      }
    }
  }

  bool canSelectSlot(int index) {
    // Slot is already occupied
    if (slotOccupancy[index]) return false;

    // Selected slots count should not exceed 3
    if (!selectedSlots[index] && selectedSlots.where((slot) => slot).length >= 3) return false;

    // Allow selecting the slot if there are no previously selected slots
    if (selectedSlots.every((slot) => !slot)) return true;

    // Check for neighboring selected slots
    var hasNeighbor = false;
    if (index > 0 && selectedSlots[index - 1]) hasNeighbor = true;
    if (index < selectedSlots.length - 1 && selectedSlots[index + 1]) hasNeighbor = true;

    return hasNeighbor;
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      showBackButton: true,
      title: 'Modul Setup',
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const Expanded(
            child: SizedBox(),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.only(top: 28, bottom: 28),
              child: Selector<ModuleProvider, List<Submodule>>(
                selector: (_, provider) => provider.submodules,
                builder: (context, submodules, child) {
                  final modules = <Submodule>[];
                  for (final submodule in submodules) {
                    if (!modules.any((module) => module.address.moduleID == submodule.address.moduleID)) {
                      modules.add(submodule);
                    }
                  }
                  updateSlots(modules);
                  return Column(
                    children: List.generate(
                      8,
                      (index) => Expanded(
                        child: Padding(
                          padding: const EdgeInsets.all(4),
                          child: InkWell(
                            onTap: () {
                              if (canSelectSlot(index)) {
                                setState(() {
                                  selectedSlots[index] = !selectedSlots[index];
                                });
                              } else {
                                setState(() {
                                  selectedSlots[index] = false;
                                });
                              }
                              var hasUpperNeighbor = false;
                              var hasLowerNeighbor = false;
                              if (index > 0 && selectedSlots[index - 1]) hasUpperNeighbor = true;
                              if (index < selectedSlots.length - 1 && selectedSlots[index + 1]) hasLowerNeighbor = true;
                              if (hasUpperNeighbor && hasLowerNeighbor) {
                                selectedSlots[index - 1] = false;
                                selectedSlots[index + 1] = false;
                              }
                            },
                            child: Container(
                              width: double.infinity,
                              decoration: BoxDecoration(
                                borderRadius: BorderRadius.circular(8),
                                color: slotOccupancy[index] ? Colors.black.withOpacity(0.2) : RobotColors.accent,
                                border: selectedSlots[index] ? Border.all(width: 5) : null,
                              ),
                              child: Padding(
                                padding: const EdgeInsets.all(8.0),
                                child: Center(
                                  child: Text(
                                    slotOccupancy[index] ? 'Belegt' : 'Frei',
                                    style: const TextStyle(color: RobotColors.primaryText, fontSize: 28),
                                  ),
                                ),
                              ),
                            ),
                          ),
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.all(32),
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.end,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('Zum Einrichten eines Moduls bitte entsprechende Slots auswählen und Modulinformationen angeben.',
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 32)),
                    const SizedBox(
                      height: 8,
                    ),
                    TextField(
                      style: const TextStyle(color: RobotColors.secondaryText, fontSize: 26),
                      controller: controller,
                      decoration: const InputDecoration(
                        hintText: 'Modul ID',
                      ),
                    ),
                    const SizedBox(
                      height: 8,
                    ),
                    Row(
                      children: [
                        Checkbox(
                          activeColor: RobotColors.accent,
                          checkColor: RobotColors.secondaryText,
                          value: isElectric,
                          onChanged: (value) {
                            setState(() {
                              isElectric = value ?? false;
                              isPartial = false;
                            });
                          },
                        ),
                        const Text(
                          'Ist elektrisch',
                          style: TextStyle(color: RobotColors.primaryText),
                        ),
                      ],
                    ),
                    const SizedBox(
                      height: 8,
                    ),
                    Row(
                      children: [
                        Checkbox(
                          activeColor: RobotColors.accent,
                          checkColor: RobotColors.secondaryText,
                          value: isPartial,
                          onChanged: (value) {
                            setState(() {
                              isPartial = value ?? false;
                              isElectric = false;
                            });
                          },
                        ),
                        const Text(
                          'Ist partiell',
                          style: TextStyle(color: RobotColors.primaryText),
                        ),
                      ],
                    ),
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(
                          child: RoundedButton(
                            onPressed: () async {
                              if (selectedSlots.every((slot) => !slot)) return;
                              final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
                              if (isPartial) {
                                for (int submoduleID = 1; submoduleID <= 8; submoduleID++) {
                                  await moduleProvider.createSubmodule(
                                    robotName: 'rb_theron',
                                    submoduleAddress: SubmoduleAddress(
                                      moduleID: int.tryParse(controller.text) ?? 0,
                                      submoduleID: submoduleID,
                                    ),
                                    position: selectedSlots.indexWhere((slot) => slot) + 1,
                                    size: selectedSlots.where((slot) => slot).length,
                                    variant: 'partial',
                                  );
                                }
                              } else {
                                await moduleProvider.createSubmodule(
                                  robotName: 'rb_theron',
                                  submoduleAddress: SubmoduleAddress(
                                    moduleID: int.tryParse(controller.text) ?? 0,
                                    submoduleID: 1,
                                  ),
                                  position: selectedSlots.indexWhere((slot) => slot) + 1,
                                  size: selectedSlots.where((slot) => slot).length,
                                  variant: isElectric ? 'electric' : 'manual',
                                );
                              }

                              selectedSlots.fillRange(0, selectedSlots.length, false);
                              isElectric = false;
                              isPartial = false;

                              await moduleProvider.fetchModules();
                            },
                            child: const Padding(
                              padding: EdgeInsets.all(8.0),
                              child: Text(
                                'Erstellen',
                                style: TextStyle(fontSize: 28, color: RobotColors.primaryText),
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(
                          width: 8,
                        ),
                        Expanded(
                          child: RoundedButton(
                            onPressed: () async {
                              final modulesProvider = Provider.of<ModuleProvider>(context, listen: false);
                              for (final submodule in modulesProvider.submodules) {
                                await modulesProvider.deleteSubmodule(
                                  robotName: 'rb_theron',
                                  submoduleAddress: submodule.address,
                                );
                              }
                              selectedSlots.fillRange(0, selectedSlots.length, false);
                            },
                            child: const Padding(
                              padding: EdgeInsets.all(8.0),
                              child: Text('Zurücksetzen', style: TextStyle(fontSize: 28, color: RobotColors.primaryText)),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
