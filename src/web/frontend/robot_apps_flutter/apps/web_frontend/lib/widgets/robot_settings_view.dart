import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/widgets/invalid_inputs_dialog.dart';

class RobotSettingsView extends StatefulWidget {
  const RobotSettingsView({required this.robotName, super.key});

  final String robotName;

  @override
  State<RobotSettingsView> createState() => _RobotSettingsViewState();
}

class _RobotSettingsViewState extends State<RobotSettingsView> {
  late Future<void> loadCleaningCycle;
  bool isUpdatingCycleValue = false;
  final cycleController = TextEditingController();
  final disinfectionRequirements = {
    'Vor Modulöffnung': true,
    'Vor Touchscreennutzung': true,
    'Nach Touchscreennutzung': true,
  };

  Future<void> initCleaningCycle() async {
    final cycle = await MiddlewareApiUtilities().hygiene.getCycle(robotName: widget.robotName);
    cycleController.text = cycle.toString();
  }

  @override
  void initState() {
    super.initState();
    loadCleaningCycle = initCleaningCycle();
  }

  Future<void> updateCleaningCycle(double cycle) async {
    await MiddlewareApiUtilities().hygiene.setCycle(robotName: widget.robotName, cycleTimeInH: cycle);
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: FutureBuilder<void>(
        future: loadCleaningCycle,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const CircularProgressIndicator();
          }
          return Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Reinigungszyklus',
                style: TextStyle(fontSize: 24, color: WebColors.secondaryText),
              ),
              const SizedBox(height: 8),
              Row(
                children: [
                  const Text(
                    'Zykluszeit in Stunden:',
                    style: TextStyle(color: WebColors.secondaryText, fontSize: 18),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: TextFormField(
                      controller: cycleController,
                      style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
                    ),
                  ),
                  const SizedBox(width: 8),
                  IconButton(
                      iconSize: 24,
                      onPressed: () async {
                        setState(() {
                          isUpdatingCycleValue = true;
                        });
                        final value = double.tryParse(cycleController.text);
                        if (value == null) {
                          await showDialog<InvalidInputsDialog>(context: context, builder: (context) => const InvalidInputsDialog());
                        } else {
                          final fleetProvider = Provider.of<FleetProvider>(context, listen: false);
                          await updateCleaningCycle(value);
                          await fleetProvider.updateRobots();
                        }
                        setState(() {
                          isUpdatingCycleValue = false;
                        });
                      },
                      icon: isUpdatingCycleValue ? const Icon(Icons.refresh) : const Icon(Icons.save),),
                ],
              ),
              const SizedBox(height: 8),
              const Text(
                'Desinfektionsanforderungen',
                style: TextStyle(fontSize: 24, color: WebColors.secondaryText),
              ),
              const SizedBox(height: 4),
              Stack(
                alignment: Alignment.bottomRight,
                children: [
                  Column(
                    children: disinfectionRequirements.entries.map(
                      (entry) {
                        return Row(
                          children: [
                            Checkbox(
                              value: entry.value,
                              onChanged: (value) {
                                setState(() {
                                  disinfectionRequirements[entry.key] = value!;
                                });
                              },
                            ),
                            Text(
                              entry.key,
                              style: const TextStyle(color: WebColors.secondaryText, fontSize: 18),
                            ),
                          ],
                        );
                      },
                    ).toList(),
                  ),
                  IconButton(
                    onPressed: () {},
                    icon: const Icon(Icons.save),
                    iconSize: 24,
                  ),
                ],
              ),
            ],
          );
        },
      ),
    );
  }
}
