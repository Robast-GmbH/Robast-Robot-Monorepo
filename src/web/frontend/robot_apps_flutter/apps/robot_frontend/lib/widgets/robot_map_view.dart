import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:shared_widgets/shared_widgets.dart';
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
            if (snapshot.connectionState != ConnectionState.done || snapshot.data == null) {
              return const Center(child: CircularProgressIndicator());
            }
            return Zoom(
              canvasColor: Colors.transparent,
              backgroundColor: Colors.transparent,
              canvasColor: Colors.transparent,
              backgroundColor: Colors.transparent,
              initTotalZoomOut: true,
              child: SizedBox(
                width: controller.mapWidth.toDouble(),
                height: controller.mapHeight.toDouble(),
                child: GestureDetector(
                  onTapDown: (details) {
                    if (widget.onTap != null) {
                      final mapFramePosition = controller.calculateNavGoal(pose: Pose(x: details.localPosition.dx, y: details.localPosition.dy));
                      widget.onTap!(mapFramePosition);
                      setState(() {
                        mapMarkerPosition = details.localPosition;
                      });
                    }
                  },
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
                        FutureBuilder<void>(
                          future: Provider.of<RobotProvider>(context, listen: false).updateRobotPose(),
                          builder: (context, snapshot) {
                            final pose = Provider.of<RobotProvider>(context, listen: false).robotPose;
                            if (snapshot.connectionState != ConnectionState.done || pose == null) {
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
                      if (mapMarkerPosition != null)
                        Positioned(
                          top: mapMarkerPosition!.dy - 40,
                          left: mapMarkerPosition!.dx - 40,
                          child: Container(
                            width: 80,
                            height: 80,
                            decoration: const BoxDecoration(
                              color: RobotColors.accent,
                              shape: BoxShape.circle,
                            ),
                            child: const Icon(
                            child: const Icon(
                              Icons.pin_drop,
                              color: RobotColors.primaryIcon,
                              size: 40,
                            ),
                          ),
                        ),
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
}
