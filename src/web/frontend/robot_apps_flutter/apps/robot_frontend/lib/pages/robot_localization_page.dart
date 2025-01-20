import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/buttons/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/robot_map_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class RobotLocalizationPage extends StatefulWidget {
  const RobotLocalizationPage({super.key});

  @override
  State<RobotLocalizationPage> createState() => _RobotLocalizationPageState();
}

class _RobotLocalizationPageState extends State<RobotLocalizationPage> {
  Pose? _lastTapPosition;
  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      inactivityTimerEnabled: false,
      title: 'Roboter Lokalisierung',
      child: Stack(
        alignment: Alignment.bottomCenter,
        children: [
          RobotMapView(
            displayRobotPose: false,
            onTap: (position) {
              setState(() {});
              _lastTapPosition = position;
            },
          ),
          Padding(
            padding: const EdgeInsets.only(bottom: 16),
            child: CustomElevatedButton(
              onPressed: () {},
              label: 'Position bestätigen',
              enabled: _lastTapPosition != null,
            ),
          )
        ],
      ),
    );
  }
}
