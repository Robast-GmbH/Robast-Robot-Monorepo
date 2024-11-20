import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/pages/manuals_page.dart';

class EmergencyStopView extends StatelessWidget {
  const EmergencyStopView({super.key});

  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      color: RobotColors.error,
      child: Stack(
        fit: StackFit.expand,
        children: [
          const Center(
            child: Text(
              "Not-Aus Schalter gedrückt",
              style: TextStyle(color: RobotColors.secondaryText, fontSize: 80),
            ),
          ),
          Align(
            alignment: Alignment.topLeft,
            child: IconButton(
              icon: const Icon(Icons.info_outline),
              iconSize: 40,
              color: RobotColors.primaryIcon,
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (context) => const ManualsPage(),
                  ),
                );
              },
            ),
          )
        ],
      ),
    );
  }
}
