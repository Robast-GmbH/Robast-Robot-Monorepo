import 'dart:math';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:zoom_widget/zoom_widget.dart';

class RobotMapView extends StatefulWidget {
  const RobotMapView({this.displayRobotPose = true, this.onTap, super.key});

  final bool displayRobotPose;
  final void Function(Pose)? onTap;

  @override
  State<RobotMapView> createState() => _RobotMapViewState();
}

class _RobotMapViewState extends State<RobotMapView> {
  late Future<Uint8List?> loadMapImageFuture;
  final MapController controller = MapController(mapPath: '');
  Offset? mapMarkerPosition;
  Offset? directionMarkerPosition;
  double? yaw;
  bool showDirectionIndicator = false;
  Function(void Function())? setMarkerState;

  @override
  void initState() {
    super.initState();
    loadMapImageFuture = Provider.of<MapProvider>(context, listen: false).getMapImage();
  }

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(16),
      child: Container(
        width: double.infinity,
        height: double.infinity,
        decoration: BoxDecoration(color: Colors.black.withOpacity(0.2)),
        child: FutureBuilder<Uint8List?>(
          future: loadMapImageFuture,
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done || snapshot.data == null) {
              return const Center(child: CircularProgressIndicator());
            }
            return Zoom(
              canvasColor: Colors.transparent,
              backgroundColor: Colors.transparent,
              initTotalZoomOut: true,
              child: SizedBox(
                width: controller.mapWidth.toDouble(),
                height: controller.mapHeight.toDouble(),
                child: _buildMapMarkerDetector(
                  shouldDetect: widget.onTap != null,
                  child: Stack(
                    alignment: Alignment.center,
                    children: [
                      Container(
                        decoration: BoxDecoration(
                          image: DecorationImage(
                            image: MemoryImage(snapshot.data!),
                          ),
                        ),
                      ),
                      if (widget.displayRobotPose)
                        Selector<RobotProvider, Pose?>(
                          selector: (_, provider) => provider.robotPose,
                          builder: (context, pose, child) {
                            if (pose == null) {
                              return const SizedBox();
                            }
                            final rmfPose = controller.calculateMapPosition(pose: pose);
                            return Positioned(
                              top: -rmfPose.y - 40,
                              left: rmfPose.x - 40,
                              child: Container(
                                width: 80,
                                height: 80,
                                decoration: const BoxDecoration(
                                  color: RobotColors.accent,
                                  shape: BoxShape.circle,
                                ),
                                child: const Icon(
                                  Icons.pin_drop,
                                  color: RobotColors.primaryIcon,
                                  size: 40,
                                ),
                              ),
                            );
                          },
                        ),
                      if (widget.onTap != null)
                        StatefulBuilder(builder: (context, innerSetMarkerState) {
                          setMarkerState = innerSetMarkerState;
                          return Stack(
                            children: [
                              if (mapMarkerPosition != null)
                                Positioned(
                                  top: mapMarkerPosition!.dy - 40,
                                  left: mapMarkerPosition!.dx - 40,
                                  child: Transform.rotate(
                                    angle: -(yaw ?? 0.0),
                                    child: Container(
                                      width: 80,
                                      height: 80,
                                      decoration: const BoxDecoration(
                                        color: RobotColors.accent,
                                        shape: BoxShape.circle,
                                      ),
                                      child: const Icon(
                                        Icons.arrow_forward,
                                        color: RobotColors.primaryIcon,
                                        size: 40,
                                      ),
                                    ),
                                  ),
                                ),
                              if (mapMarkerPosition != null && directionMarkerPosition != null)
                                PositionedLine(
                                  from: mapMarkerPosition!,
                                  to: directionMarkerPosition!,
                                  color: RobotColors.accent,
                                  strokeWidth: 3.0,
                                ),
                            ],
                          );
                        }),
                    ],
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );
  }

  _buildMapMarkerDetector({required Widget child, required bool shouldDetect}) {
    if (!shouldDetect) return child;
    return GestureDetector(
      onLongPressStart: (details) {
        setMarkerState?.call(() {
          mapMarkerPosition = details.localPosition;
        });
      },
      onLongPressMoveUpdate: (details) {
        directionMarkerPosition = details.localPosition;
        final vectorA = [
          mapMarkerPosition!.dx - directionMarkerPosition!.dx,
          mapMarkerPosition!.dy - directionMarkerPosition!.dy,
        ];
        final vectorB = [1, 0];
        final angle = acos(vectorA[0] * vectorB[0] / (sqrt(pow(vectorA[0], 2) + pow(vectorA[1], 2))));
        yaw = (mapMarkerPosition!.dy > directionMarkerPosition!.dy ? -1.0 * angle : angle) + pi;

        setMarkerState?.call(() {});
      },
      onLongPressEnd: (details) => setMarkerState?.call(() {
        final mapFramePosition = controller.calculateNavGoal(pose: Pose(x: details.localPosition.dx, y: -details.localPosition.dy, yaw: yaw!));
        widget.onTap?.call(mapFramePosition);
        directionMarkerPosition = null;
      }),
      onLongPressCancel: () => setMarkerState?.call(() {
        directionMarkerPosition = null;
      }),
      child: child,
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
