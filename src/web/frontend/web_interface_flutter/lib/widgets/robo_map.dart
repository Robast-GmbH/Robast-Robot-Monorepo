import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';

import 'package:web_interface_flutter/models/map_controller.dart';
import 'package:web_interface_flutter/models/data/robot.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/widgets/robot_marker.dart';

import 'package:zoom_widget/zoom_widget.dart';

class RoboMap extends StatefulWidget {
  const RoboMap({
    super.key,
    required this.controller,
    this.isMultiTarget = false,
  });
  final MapController controller;
  final bool isMultiTarget;

  @override
  State<RoboMap> createState() => _RoboMapState();
}

class _RoboMapState extends State<RoboMap> {
  Offset tempPos = const Offset(0, 0);
  bool showDirectionIndicator = false;
  Offset? startPosDirection;
  Offset? endPosDirection;
  int counter = 0;
  bool allowScroll = true;

  @override
  Widget build(BuildContext context) {
    final robotOrigin = widget.controller.getRobotOriginAsOffset();
    return Selector<RobotProvider, List<Robot>>(
      selector: (_, provider) => provider.robots,
      builder: (context, robots, child) {
        return ClipRRect(
          borderRadius: BorderRadius.circular(16),
          child: RotatedBox(
            quarterTurns: kIsWeb ? 1 : 0,
            child: Stack(
              alignment: Alignment.topCenter,
              children: [
                Zoom(
                  backgroundColor: AppColors.lightGrey,
                  maxScale: 5,
                  initTotalZoomOut: true,
                  enableScroll: !showDirectionIndicator,
                  child: SizedBox(
                    width: widget.controller.mapWidth.toDouble(),
                    height: widget.controller.mapHeight.toDouble(),
                    child: GestureDetector(
                      onTapDown: (details) {
                        tempPos = details.localPosition;
                      },
                      onLongPressStart: (details) {
                        showDirectionIndicator = true;
                        startPosDirection = details.localPosition;
                        endPosDirection = details.localPosition;
                        setState(() {});
                      },
                      onLongPressUp: () {
                        showDirectionIndicator = false;
                        setState(() {});
                      },
                      onLongPressMoveUpdate: (details) {
                        endPosDirection = details.localPosition;
                        final vectorA = [
                          startPosDirection!.dx - endPosDirection!.dx,
                          startPosDirection!.dy - endPosDirection!.dy,
                        ];
                        final vectorB = [1, 0];
                        final angle = acos(vectorA[0] * vectorB[0] / (sqrt(pow(vectorA[0], 2) + pow(vectorA[1], 2))));
                        widget.controller.yaws.last = (startPosDirection!.dy > endPosDirection!.dy ? -1.0 * angle : angle) + pi;

                        setState(() {});
                      },
                      onLongPressCancel: () {
                        showDirectionIndicator = false;
                        setState(() {});
                      },
                      onLongPressEnd: (details) {
                        showDirectionIndicator = false;
                        setState(() {});
                      },
                      onLongPress: () {
                        if (!widget.isMultiTarget) {
                          widget.controller.points.clear();
                          widget.controller.yaws.clear();
                        }
                        widget.controller.points.add(tempPos);

                        widget.controller.yaws.add(0);
                        setState(() {});
                      },
                      child: Stack(
                        children: [
                          Image.asset("assets/small_sim_map.png"),
                          ...List.generate(
                            widget.controller.points.length,
                            (index) => Positioned(
                              left: widget.controller.points[index].dx - 12,
                              top: widget.controller.points[index].dy - 12,
                              child: Transform.rotate(
                                angle: -widget.controller.yaws[index],
                                child: GestureDetector(
                                  onLongPress: () {
                                    if (widget.isMultiTarget) {
                                      widget.controller.points.removeAt(index);
                                      widget.controller.yaws.removeAt(index);
                                    }
                                    setState(() {});
                                  },
                                  child: Container(
                                    decoration: const BoxDecoration(
                                      borderRadius: BorderRadius.all(
                                        Radius.circular(4),
                                      ),
                                      color: Colors.orange,
                                    ),
                                    child: widget.isMultiTarget
                                        ? Padding(
                                            padding: Constants.tinyPadding,
                                            child: Text("${index < 10 ? 0 : ''}${index.toString()}"),
                                          )
                                        : const Icon(
                                            Icons.arrow_forward,
                                            size: 24,
                                          ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          ...robots
                              .map(
                                (robot) => Positioned(
                                  left: robotOrigin.dx - 12 + robot.x / widget.controller.resolution,
                                  top: robotOrigin.dy - 12 - robot.y / widget.controller.resolution,
                                  child: Transform.rotate(
                                    angle: -robot.yaw,
                                    child: const RobotMarker(),
                                  ),
                                ),
                              )
                              .toList(),
                          if (showDirectionIndicator) ...[
                            PositionedLine(
                              from: startPosDirection!,
                              to: endPosDirection!,
                              color: Colors.red,
                              strokeWidth: 2,
                            ),
                          ]
                        ],
                      ),
                    ),
                  ),
                ),
                if (widget.controller.points.isEmpty) ...[
                  const Padding(
                    padding: Constants.smallPadding,
                    child: Text("Zur Zielauswahl lange drücken"),
                  )
                ],
              ],
            ),
          ),
        );
      },
    );
  }
}

class PositionedLine extends StatelessWidget {
  final Offset from;
  final Offset to;
  final Color color;
  final double strokeWidth;

  const PositionedLine({
    super.key,
    required this.from,
    required this.to,
    this.color = Colors.black,
    this.strokeWidth = 1.0,
  });

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      painter: LinePainter(from, to, color, strokeWidth),
      child: Container(),
    );
  }
}

class LinePainter extends CustomPainter {
  final Offset from;
  final Offset to;
  final Color color;
  final double strokeWidth;

  LinePainter(this.from, this.to, this.color, this.strokeWidth);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = color
      ..strokeWidth = strokeWidth;

    canvas.drawLine(from, to, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}
