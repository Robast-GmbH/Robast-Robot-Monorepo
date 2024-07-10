import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/titled_view.dart';

class DrawerSetupPage extends StatefulWidget {
  const DrawerSetupPage({super.key});

  @override
  State<DrawerSetupPage> createState() => _DrawerSetupPageState();
}

class _DrawerSetupPageState extends State<DrawerSetupPage> {
  final List<bool> slotOccupancy = List.generate(8, (index) => false);
  final List<bool> selectedSlots = List.generate(8, (index) => false);
  final controller = TextEditingController();
  bool isElectric = false;

  void updateSlots(List<RobotDrawer> drawers) {
    slotOccupancy.fillRange(0, slotOccupancy.length, false);
    for (final drawer in drawers) {
      for (var i = 0; i < drawer.size; i++) {
        slotOccupancy[drawer.position + i - 1] = true;
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
    return Scaffold(
      body: BackgroundView(
        child: TitledView(
          showBackButton: true,
          title: 'Drawer Setup',
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Expanded(
                child: SizedBox(),
              ),
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.only(top: 16, bottom: 32),
                  child: Selector<ModuleProvider, List<RobotDrawer>>(
                    selector: (_, provider) => provider.modules,
                    builder: (context, modules, child) {
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
                                  decoration: BoxDecoration(
                                    color: slotOccupancy[index] ? Colors.grey : Colors.greenAccent,
                                    border: selectedSlots[index] ? Border.all(width: 5) : null,
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
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.end,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Checkbox(
                            value: isElectric,
                            onChanged: (value) {
                              setState(() {
                                isElectric = value ?? false;
                              });
                            },
                          ),
                          const Text('Ist elektrisch'),
                        ],
                      ),
                      TextField(
                        controller: controller,
                        decoration: const InputDecoration(
                          hintText: 'Module ID',
                        ),
                      ),
                      const SizedBox(height: 16),
                      Row(
                        children: [
                          TextButton.icon(
                            onPressed: () async {
                              if (selectedSlots.every((slot) => !slot)) return;
                              await Provider.of<ModuleProvider>(context, listen: false).createModule(
                                robotName: 'rb_theron',
                                moduleID: int.tryParse(controller.text) ?? 0,
                                drawerID: 0,
                                position: selectedSlots.indexWhere((slot) => slot) + 1,
                                size: selectedSlots.where((slot) => slot).length,
                                variant: isElectric ? 'electric' : 'manual',
                              );
                              selectedSlots.fillRange(0, selectedSlots.length, false);
                              isElectric = false;
                              if (context.mounted) {
                                await Provider.of<ModuleProvider>(context, listen: false).fetchModules();
                              }
                            },
                            label: const Text('Add'),
                            icon: const Icon(Icons.add),
                          ),
                          TextButton.icon(
                            onPressed: () async {
                              final modulesProvider = Provider.of<ModuleProvider>(context, listen: false);
                              for (final drawer in modulesProvider.modules) {
                                await modulesProvider.deleteModule(
                                  robotName: 'rb_theron',
                                  moduleID: drawer.moduleID,
                                  drawerID: drawer.drawerID,
                                );
                              }
                              selectedSlots.fillRange(0, selectedSlots.length, false);
                            },
                            label: const Text('Delete all'),
                            icon: const Icon(Icons.delete),
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
      ),
    );
  }
}
