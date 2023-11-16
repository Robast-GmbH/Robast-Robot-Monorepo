import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/robo_map.dart';

class PatrolOverview extends StatefulWidget {
  const PatrolOverview({super.key});

  @override
  State<PatrolOverview> createState() => _PatrolOverviewState();
}

class _PatrolOverviewState extends State<PatrolOverview> {
  final controller = MapController();
  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        RoboMap(
          controller: controller,
          isMultiTarget: true,
        ),
        Padding(
          padding: Constants.mediumPadding,
          child: Align(
            alignment: Alignment.bottomRight,
            child: FloatingActionButton(
              onPressed: () async {
                if (controller.points.length >= 2) {
                  await APIService.startPatrol(
                    robotName: "ROBAST",
                    fleetName: "RB0",
                    points: controller.positionsAsNavPoints(),
                    yaws: controller.yaws,
                    ownerID: 2,
                    taskID: Provider.of<RobotProvider>(context, listen: false).generateTaskID(),
                  );
                }
              },
              child: const Icon(Icons.start),
            ),
          ),
        ),
      ],
    );
  }
}
