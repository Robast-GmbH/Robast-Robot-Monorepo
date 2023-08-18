import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/robo_map.dart';

class ManualMoveMap extends StatefulWidget {
  const ManualMoveMap({super.key});

  @override
  State<ManualMoveMap> createState() => _ManualMoveMapState();
}

class _ManualMoveMapState extends State<ManualMoveMap> {
  final ownerID = 2;
  final yaw = 0.0;
  Timer? refreshTimer;
  void setTimer() {
    refreshTimer = Timer.periodic(const Duration(seconds: 5), (timer) async {
      debugPrint("Update Robots");
      await Provider.of<RobotProvider>(context, listen: false).updateRobots();
    });
  }

  @override
  void initState() {
    super.initState();
    setTimer();
  }

  @override
  void dispose() {
    refreshTimer?.cancel();
    super.dispose();
  }

  final controller = MapController();
  @override
  Widget build(BuildContext context) {
    return Stack(
      alignment: Alignment.bottomRight,
      children: [
        RoboMap(controller: controller),
        Padding(
          padding: const EdgeInsets.all(16),
          child: FloatingActionButton(
            onPressed: () async {
              await APIService.moveRobot(
                "RB0",
                controller.position!.dx,
                controller.position!.dy,
                yaw,
                ownerID,
              );
            },
            child: const Icon(Icons.check),
          ),
        )
      ],
    );
  }
}
