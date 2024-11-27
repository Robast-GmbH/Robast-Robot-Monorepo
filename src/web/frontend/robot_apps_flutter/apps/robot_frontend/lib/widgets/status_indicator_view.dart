import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/pages/setting_pages/config_page.dart';
import 'package:robot_frontend/widgets/buttons/developer_button_view.dart';

class StatusIndicatorView extends StatefulWidget {
  const StatusIndicatorView({this.shouldBlockNavigation = false, super.key});

  final bool shouldBlockNavigation;

  @override
  State<StatusIndicatorView> createState() => _StatusIndicatorViewState();
}

class _StatusIndicatorViewState extends State<StatusIndicatorView> {
  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicBatteryLevelUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicBatteryLevelUpdate();
    super.deactivate();
  }

  IconData getBatteryIcon(double? batteryLevel) {
    if (batteryLevel == null) {
      return Icons.battery_unknown;
    } else if (batteryLevel >= 95) {
      return Icons.battery_full;
    } else if (batteryLevel >= 90) {
      return Icons.battery_6_bar;
    } else if (batteryLevel >= 70) {
      return Icons.battery_5_bar;
    } else if (batteryLevel >= 50) {
      return Icons.battery_4_bar;
    } else if (batteryLevel >= 30) {
      return Icons.battery_3_bar;
    } else if (batteryLevel >= 15) {
      return Icons.battery_2_bar;
    } else if (batteryLevel >= 5) {
      return Icons.battery_1_bar;
    } else {
      return Icons.battery_0_bar;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        const Icon(
          Icons.link,
          size: 40,
        ),
        const SizedBox(width: 8),
        RotatedBox(
          quarterTurns: 1,
          child: Selector<RobotProvider, double?>(
              selector: (context, provider) => provider.batteryLevel,
              builder: (context, batteryLevel, child) {
                return Icon(
                  getBatteryIcon(batteryLevel),
                  size: 40,
                );
              }),
        ),
        const SizedBox(width: 8),
        IconButton(
          padding: const EdgeInsets.all(0),
          onPressed: () async {
            final robotProvider = Provider.of<RobotProvider>(context, listen: false);
            if (widget.shouldBlockNavigation) {
              robotProvider.blockNavigation();
            }
            final inactivityProvider = Provider.of<InactivityProvider>(context, listen: false);
            final isActive = inactivityProvider.isActive();
            await Navigator.push(context, MaterialPageRoute<ManualsPage>(builder: (context) => const ManualsPage()));
            if (!isActive) {
              inactivityProvider.cancelTimer();
            }
            if (widget.shouldBlockNavigation) {
              robotProvider.unblockNavigation();
            }
          },
          color: RobotColors.primaryIcon,
          icon: const Icon(
            Icons.info_outline,
            size: 36,
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
  }
}
