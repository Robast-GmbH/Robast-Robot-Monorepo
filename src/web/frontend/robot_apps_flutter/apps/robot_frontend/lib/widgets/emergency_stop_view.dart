import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:robot_frontend/widgets/video_view.dart';

class EmergencyStopView extends StatelessWidget {
  const EmergencyStopView({super.key});

  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      color: RobotColors.error,
      child: Stack(
        fit: StackFit.expand,
        children: [
          Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Icon(Icons.warning_amber_outlined, size: 120, color: RobotColors.primaryIcon),
                const Text(
                  "Not\u{2011}Aus aktiviert",
                  style: TextStyle(color: RobotColors.secondaryText, fontSize: 100),
                ),
                const SizedBox(
                  height: 16,
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Row(
                    children: [
                      Expanded(
                        child: buildEmergencyStopInfoCard(
                            text: 'Zum Deaktivieren die Not\u{2011}Aus\u{2011}Schalter entriegeln und den blauen Knopf drücken.',
                            videoPath: 'assets/release_emergency_stop.MP4'),
                      ),
                      Expanded(
                        child: buildEmergencyStopInfoCard(
                            text: 'Zum Verschieben des Roboters den orangenen Knopf gedrückt halten.', videoPath: 'assets/move_robot.MP4'),
                      ),
                    ],
                  ),
                ),
                const SizedBox(
                  height: 16,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Padding buildEmergencyStopInfoCard({required String text, required String videoPath}) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: RoundedContainer(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const SizedBox(height: 8),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 32),
              child: Text(
                text,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  color: RobotColors.secondaryText,
                  fontSize: 32,
                  height: 1.75,
                ),
              ),
            ),
            const SizedBox(height: 8),
            AspectRatio(
              aspectRatio: 16 / 9,
              child: ClipRRect(
                borderRadius: BorderRadius.circular(16),
                child: VideoView(
                  path: videoPath,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
