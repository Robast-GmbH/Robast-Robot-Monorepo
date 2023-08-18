import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';

import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/robot.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/widgets/robot_marker.dart';

import 'package:zoom_widget/zoom_widget.dart';

class RoboMap extends StatefulWidget {
  const RoboMap({super.key, required this.controller});
  final MapController controller;

  @override
  State<RoboMap> createState() => _RoboMapState();
}

class _RoboMapState extends State<RoboMap> {
  Offset tempPos = const Offset(0, 0);
  Offset mapOrigin = const Offset(294, 99);
  Offset examplePoint = const Offset(29.2, -57.8);
  final double resolution = 0.05;
  @override
  Widget build(BuildContext context) {
    return Selector<RobotProvider, List<Robot>>(
      selector: (_, provider) => provider.robots,
      builder: (context, robots, child) {
        return Stack(
          alignment: Alignment.topCenter,
          children: [
            Zoom(
              backgroundColor: AppColors.lightGrey,
              maxScale: 5,
              initTotalZoomOut: true,
              child: SizedBox(
                width: 827,
                height: 355,
                child: GestureDetector(
                  onTapDown: (details) {
                    tempPos = details.localPosition;
                  },
                  onLongPress: () {
                    widget.controller.position = tempPos;
                    setState(() {});
                  },
                  child: Stack(
                    children: [
                      Image.asset("assets/RL_Tiplu_6.png"),
                      if (widget.controller.position != null) ...[
                        Positioned(
                          left: widget.controller.position!.dx - 8,
                          top: widget.controller.position!.dy - 8,
                          child: Container(
                            decoration: const BoxDecoration(
                              borderRadius: BorderRadius.all(Radius.circular(4)),
                              color: Colors.orange,
                            ),
                            child: const Icon(
                              Icons.clear,
                              size: 16,
                            ),
                          ),
                        ),
                      ],
                      ...robots
                          .map((robot) => Positioned(
                                left: mapOrigin.dx - 8 + robot.x / resolution,
                                top: mapOrigin.dy - 8 - robot.y / resolution,
                                child: Transform.rotate(angle: -1.57, child: const RobotMarker()),
                              ))
                          .toList(),
                    ],
                  ),
                ),
              ),
            ),
            if (widget.controller.position == null) ...[
              const Padding(
                padding: EdgeInsets.all(8.0),
                child: Text("Zur Zielauswahl lange drücken"),
              )
            ]
          ],
        );
      },
    );
  }
}
