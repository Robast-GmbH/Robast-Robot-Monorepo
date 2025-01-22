import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/setting_pages/config_page.dart';
import 'package:robot_frontend/widgets/buttons/developer_button_view.dart';

class StatusIndicatorView extends StatefulWidget {
  const StatusIndicatorView({this.shouldBlockNavigation = false, super.key});

  final bool shouldBlockNavigation;

  @override
  State<StatusIndicatorView> createState() => _StatusIndicatorViewState();
}

class _StatusIndicatorViewState extends State<StatusIndicatorView> {
  final maxAnimationOffset = 7;
  Timer? chargingAnimationTimer;
  int? chargingIndex = 0;
  int animationOffset = 0;

  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicBatteryStatusUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicBatteryStatusUpdate();
    super.deactivate();
  }

  int? getBatteryIndexByLevel(double? batteryLevel) {
    if (batteryLevel == null) {
      return null;
    } else if (batteryLevel >= 95) {
      return 7;
    } else if (batteryLevel >= 90) {
      return 6;
    } else if (batteryLevel >= 70) {
      return 5;
    } else if (batteryLevel >= 50) {
      return 4;
    } else if (batteryLevel >= 30) {
      return 3;
    } else if (batteryLevel >= 15) {
      return 2;
    } else if (batteryLevel >= 5) {
      return 1;
    } else {
      return 0;
    }
  }

  IconData getBatteryIconByIndex(int? index) {
    if (index == null) {
      return Icons.battery_unknown;
    } else if (index >= 7) {
      return Icons.battery_full;
    } else if (index == 6) {
      return Icons.battery_6_bar;
    } else if (index == 5) {
      return Icons.battery_5_bar;
    } else if (index == 4) {
      return Icons.battery_4_bar;
    } else if (index == 3) {
      return Icons.battery_3_bar;
    } else if (index == 2) {
      return Icons.battery_2_bar;
    } else if (index == 1) {
      return Icons.battery_1_bar;
    } else {
      return Icons.battery_0_bar;
    }
  }

  void startChargingAnimation() {
    chargingAnimationTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      animationOffset += 1;
      animationOffset %= (maxAnimationOffset + 1) - (chargingIndex ?? 0);
      if (mounted) {
        setState(() {});
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Selector<RobotProvider, bool?>(
      selector: (context, provider) => provider.isCharging,
      builder: (context, isCharging, child) {
        if (!(isCharging ?? false)) {
          chargingAnimationTimer?.cancel();
          animationOffset = 0;
        } else if (chargingAnimationTimer == null || !chargingAnimationTimer!.isActive) {
          startChargingAnimation();
        }
        return Row(
          mainAxisAlignment: MainAxisAlignment.end,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            RotatedBox(
              quarterTurns: 1,
              child: Selector<RobotProvider, double?>(
                selector: (context, provider) => provider.batteryLevel,
                builder: (context, batteryLevel, child) {
                  chargingIndex = getBatteryIndexByLevel(batteryLevel);
                  return Icon(
                    getBatteryIconByIndex(chargingIndex != null ? chargingIndex! + animationOffset : null),
                    size: 40,
                  );
                },
              ),
            ),
            const SizedBox(width: 8),
            DeveloperButtonView(
              onPressed: () {
                Navigator.push(context, MaterialPageRoute<ConfigPage>(builder: (context) => const ConfigPage()));
              },
              child: Container(
                margin: const EdgeInsets.all(4),
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: const Color.fromARGB(255, 0, 226, 0),
                  border: Border.all(
                    color: RobotColors.primaryText,
                    width: 4,
                  ),
                ),
                width: 30,
                height: 30,
              ),
            ),
          ],
        );
      },
    );
  }
}
