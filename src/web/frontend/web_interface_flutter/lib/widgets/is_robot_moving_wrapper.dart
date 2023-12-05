import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class IsRobotMovingWrapper extends StatelessWidget {
  const IsRobotMovingWrapper({
    super.key,
    required this.child,
  });
  final Widget child;
  @override
  Widget build(BuildContext context) {
    return Selector<RobotProvider, bool>(
      selector: (context, provider) => provider.isRobotMoving,
      builder: (context, snapshot, widget) {
        if (snapshot) {
          return Center(
            child: Padding(
              padding: const EdgeInsets.all(256),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(64),
                child: Material(
                  color: AppColors.darkBlueAccent,
                  child: InkWell(
                    onTap: () async {
                      await APIService.pauseOrResumeRobot(robotName: "ROBAST", fleetName: "RB0", shouldPause: true);
                    },
                    child: const SizedBox(
                      width: double.infinity,
                      height: double.infinity,
                      child: Center(
                        child: Text(
                          "ANHALTEN",
                          style: TextStyle(fontSize: 128),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          );
        }
        return child;
      },
    );
  }
}
