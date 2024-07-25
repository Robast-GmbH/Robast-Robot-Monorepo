import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/robo_map.dart';

class ManualMoveMap extends StatefulWidget {
  const ManualMoveMap({super.key});

  @override
  State<ManualMoveMap> createState() => _ManualMoveMapState();
}

class _ManualMoveMapState extends State<ManualMoveMap> {
  final ownerID = 2;

  final controller = MapController();
  @override
  Widget build(BuildContext context) {
    return Stack(
      alignment: Alignment.bottomRight,
      children: [
        RoboMap(controller: controller),
        Padding(
          padding: Constants.mediumPadding,
          child: FloatingActionButton(
            backgroundColor: AppColors.turquoise,
            onPressed: () async {
              if (controller.points.isEmpty) return;

              final navPos = controller.positionsAsNavPoints().first;
              await APIService.moveRobot(
                robotName: "rb_theron",
                x: navPos.dx,
                y: navPos.dy,
                yaw: controller.yaws.first,
                ownerID: ownerID,
              );
            },
            child: const Icon(Icons.gamepad_outlined),
          ),
        )
      ],
    );
  }
}
