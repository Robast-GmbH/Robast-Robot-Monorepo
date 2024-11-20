import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class RobotStatusPage extends StatelessWidget {
  const RobotStatusPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Roboterzustand',
      child: Center(
        child: Column(
          children: [
            Text("Batterie: ${Provider.of<RobotProvider>(context).batteryLevel.toString()}%",
                style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
            Text("Verbleibende Desinfektionen: ${Provider.of<RobotProvider>(context).remainingDisinfections.toString()}",
                style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText)),
          ],
        ),
      ),
    );
  }
}
