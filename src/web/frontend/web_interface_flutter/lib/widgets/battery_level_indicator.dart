import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/gaps.dart';
import 'package:web_interface_flutter/models/data/robot.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';

class BatteryLevelIndicator extends StatelessWidget {
  const BatteryLevelIndicator({super.key});

  IconData getBatteryLevelIcon(int level) {
    switch (level) {
      case > 95:
        return Icons.battery_full;
      case > 83:
        return Icons.battery_6_bar;
      case > 66:
        return Icons.battery_5_bar;
      case > 50:
        return Icons.battery_4_bar;
      case > 33:
        return Icons.battery_3_bar;
      case > 16:
        return Icons.battery_2_bar;
      case > 0:
        return Icons.battery_1_bar;

      default:
        return Icons.battery_0_bar;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Selector<RobotProvider, List<Robot>>(
      selector: (_, provider) => provider.robots,
      builder: (context, robots, child) {
        final batteryLevel = robots.firstOrNull?.batteryLevel.toInt() ?? 83;
        return Row(
          crossAxisAlignment: CrossAxisAlignment.center,
          mainAxisAlignment: MainAxisAlignment.end,
          children: [
            Icon(
              getBatteryLevelIcon(batteryLevel),
              color: AppColors.white,
            ),
            Gaps.tinyHorizontal,
            Text(
              "$batteryLevel%",
              style: const TextStyle(color: AppColors.white, fontWeight: FontWeight.w300),
            ),
          ],
        );
      },
    );
  }
}
