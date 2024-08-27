import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:shared_widgets/shared_widgets.dart';
import 'package:zoom_widget/zoom_widget.dart';

class RobotMapView extends StatelessWidget {
  RobotMapView({super.key});
  final MapController controller = MapController(mapPath: '');

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(16),
      child: Container(
        width: double.infinity,
        height: double.infinity,
        decoration: BoxDecoration(color: Colors.black.withOpacity(0.2)),
        child: FutureBuilder<Uint8List?>(
          future: Provider.of<MapProvider>(context, listen: false).getMapImage(),
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return Center(child: const CircularProgressIndicator());
            }
            return Zoom(
              backgroundColor: Colors.black.withOpacity(0.2),
              initTotalZoomOut: true,
              child: SizedBox(
                width: controller.mapWidth.toDouble(),
                height: controller.mapHeight.toDouble(),
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    Image.memory(
                      snapshot.data!,
                    ),
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
                            decoration: BoxDecoration(
                              color: RobotColors.secondaryBackground,
                              shape: BoxShape.circle,
                            ),
                            child: Icon(
                              Icons.pin_drop,
                              color: RobotColors.primaryIcon,
                              size: 40,
                            ),
                          ),
                        );
                      },
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );
  }
}
