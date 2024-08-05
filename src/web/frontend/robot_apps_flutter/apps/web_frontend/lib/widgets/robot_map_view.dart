import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:shared_widgets/shared_widgets.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';

class RobotMapView extends StatefulWidget {
  const RobotMapView({required this.robotName, super.key});

  final String robotName;

  @override
  State<RobotMapView> createState() => _RobotMapViewState();
}

class _RobotMapViewState extends State<RobotMapView> {
  final controller = MapController(mapPath: 'assets/6OG_bp.png');

  @override
  void initState() {
    Provider.of<FleetProvider>(context, listen: false).startPeriodicRobotUpdate();
    Provider.of<FleetProvider>(context, listen: false).startPeriodicIsNavigationBlockedUpdate();
    super.initState();
  }

  @override
  void deactivate() {
    Provider.of<FleetProvider>(context, listen: false).stopPeriodicUpdates();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return Selector<FleetProvider, List<Robot>>(
      selector: (_, provider) => provider.robots,
      builder: (context, robots, child) {
        final robotProvider = Provider.of<FleetProvider>(context, listen: false);
        return Stack(
          alignment: Alignment.bottomCenter,
          children: [
            RoboMap(
              controller: controller,
              robots: robots,
            ),
            GestureDetector(
              onTap: () {
                if (robotProvider.isNavigationBlocked) {
                  robotProvider
                    ..isNavigationBlocked = false
                    ..resumeRobot(robotName: widget.robotName);
                } else {
                  robotProvider
                    ..isNavigationBlocked = true
                    ..stopRobot(robotName: widget.robotName);
                }
                setState(() {});
              },
              child: ColoredBox(
                color: Colors.red,
                child: SizedBox(
                  width: double.infinity,
                  height: 50,
                  child: Center(
                    child: Text(
                      robotProvider.isNavigationBlocked ? 'Navigation Blocked - Tap to Unblock' : 'Navigation Unblocked - Tap to Block',
                      style: const TextStyle(color: Colors.white),
                    ),
                  ),
                ),
              ),
            ),
          ],
        );
      },
    );
  }
}
