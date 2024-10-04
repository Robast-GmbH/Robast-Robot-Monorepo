import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
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
            Text(Provider.of<RobotProvider>(context).batteryLevel.toString()),
            Text(Provider.of<RobotProvider>(context).remainingDisinfections.toString()),
          ],
        ),
      ),
    );
  }
}
