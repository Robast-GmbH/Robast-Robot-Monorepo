import 'dart:math';

import 'package:flutter/material.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:shared_widgets/src/models/map_controller.dart';
import 'package:shared_widgets/src/widgets/robot_marker.dart';
import 'package:zoom_widget/zoom_widget.dart';

class RoboMap extends StatefulWidget {
  const RoboMap({
    required this.controller,
    required this.robots,
    super.key,
    this.isMultiTarget = false,
  });
  final MapController controller;
  final bool isMultiTarget;
  final List<Robot> robots;

  @override
  State<RoboMap> createState() => _RoboMapState();
}

class _RoboMapState extends State<RoboMap> {
  Offset tempPos = Offset.zero;
  bool showDirectionIndicator = false;
  Offset? startPosDirection;
  Offset? endPosDirection;
  int counter = 0;
  bool allowScroll = true;

  @override
  Widget build(BuildContext context) {
    final robotOrigin = widget.controller.getRobotOriginAsOffset();
    return Zoom(
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
            if (!angle.isNaN) {
              widget.controller.yaws.last = (startPosDirection!.dy > endPosDirection!.dy ? -1.0 * angle : angle) + pi;
              setState(() {});
            }
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
            alignment: Alignment.center,
            children: [
              Image.asset(widget.controller.mapPath),
              ...List.generate(
                widget.controller.points.length,
                (index) => Positioned(
                  left: widget.controller.points[index].dx - 24,
                  top: widget.controller.points[index].dy - 24,
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
                                padding: const EdgeInsets.all(4),
                                child: Text("${index < 10 ? 0 : ''}$index"),
                              )
                            : const Icon(
                                Icons.arrow_forward,
                                size: 48,
                              ),
                      ),
                    ),
                  ),
                ),
              ),
              ...widget.robots.map(
                (robot) => Positioned(
                  left: robotOrigin.dx, //- 12 + robot.pose.x,
                  top: robotOrigin.dy, //- 12 - robot.pose.y,
                  child: Transform.rotate(
                    angle: -robot.pose.yaw,
                    child: const RobotMarker(),
                  ),
                ),
              ),
              if (showDirectionIndicator) ...[
                PositionedLine(
                  from: startPosDirection!,
                  to: endPosDirection!,
                  color: Colors.red,
                  strokeWidth: 2,
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }
}

class PositionedLine extends StatelessWidget {
  const PositionedLine({
    required this.from,
    required this.to,
    super.key,
    this.color = Colors.black,
    this.strokeWidth = 1.0,
  });

  final Offset from;
  final Offset to;
  final Color color;
  final double strokeWidth;

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      painter: LinePainter(from, to, color, strokeWidth),
      child: Container(),
    );
  }
}

class LinePainter extends CustomPainter {
  LinePainter(this.from, this.to, this.color, this.strokeWidth);

  final Offset from;
  final Offset to;
  final Color color;
  final double strokeWidth;

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
