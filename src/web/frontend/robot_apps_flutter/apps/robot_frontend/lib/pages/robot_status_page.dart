import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/robot_map_view.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotStatusPage extends StatefulWidget {
  const RobotStatusPage({super.key});

  @override
  State<RobotStatusPage> createState() => _RobotStatusPageState();
}

class _RobotStatusPageState extends State<RobotStatusPage> {
  Pose? updatedPose;
  late final String batteryLevel;

  @override
  void initState() {
    final level = Provider.of<RobotProvider>(context, listen: false).batteryLevel;
    if (level != null) {
      batteryLevel = '${level.toString()}%';
    } else {
      batteryLevel = 'Unbekannt';
    }
    Provider.of<RobotProvider>(context, listen: false).startPeriodicRobotPoseUpdate();
    super.initState();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicRobotPoseUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Roboterzustand',
      child: Center(
        child: Column(
          children: [
            Text("Batterie: $batteryLevel", style: const TextStyle(fontSize: 28, color: RobotColors.secondaryText)),
            Selector<RobotProvider, int?>(
              selector: (_, provider) => provider.remainingDisinfections,
              builder: (_, remainingDisinfections, __) {
                return Text(
                  "Verbleibende Desinfektionen: $remainingDisinfections",
                  style: const TextStyle(fontSize: 28, color: RobotColors.secondaryText),
                );
              },
            ),
            Expanded(
              child: Stack(children: [
                Padding(
                  padding: const EdgeInsets.all(16),
                  child: RobotMapView(onTap: (pose) async {
                    updatedPose = pose;
                    setState(() {});
                  }),
                ),
                if (updatedPose != null)
                  Align(
                    alignment: Alignment.bottomRight,
                    child: Padding(
                      padding: const EdgeInsets.all(32),
                      child: CustomElevatedButton(
                        onPressed: () async {
                          final robotProvider = Provider.of<RobotProvider>(context, listen: false);
                          await robotProvider.setInitialRobotPoint(pose: updatedPose!);
                          setState(() {
                            updatedPose = null;
                          });
                        },
                        label: 'Position bestätigen',
                      ),
                    ),
                  )
                else
                  const Align(
                    alignment: Alignment.topCenter,
                    child: Padding(
                      padding: EdgeInsets.only(top: 32),
                      child: RoundedContainer(
                        color: RobotColors.secondaryBackground,
                        child: Padding(
                          padding: EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                          child: Text('Zum Bestimmen der Position die Karte lange gedrückt halten',
                              style: TextStyle(fontSize: 28, color: RobotColors.secondaryText)),
                        ),
                      ),
                    ),
                  ),
              ]),
            )
          ],
        ),
      ),
    );
  }
}
