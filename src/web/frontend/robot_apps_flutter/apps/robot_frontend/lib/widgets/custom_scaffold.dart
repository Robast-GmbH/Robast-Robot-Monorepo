import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/emergency_stop_view.dart';
import 'package:robot_frontend/widgets/status_bar.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';
import 'package:robot_frontend/widgets/titled_view.dart';

class CustomScaffold extends StatelessWidget {
  const CustomScaffold({
    super.key,
    this.child = const SizedBox(),
    this.showBackButton = true,
    this.inactivityTimerEnabled = true,
    this.onBackButtonPressed,
    this.title = '',
    this.ignoreEmergencyStop = false,
    this.collapsedTitle = false,
  });

  final Widget child;
  final bool showBackButton;
  final bool inactivityTimerEnabled;
  final VoidCallback? onBackButtonPressed;
  final String title;
  final bool ignoreEmergencyStop;
  final bool collapsedTitle;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        body: BackgroundView(
      inactivityTimerEnabled: inactivityTimerEnabled,
      child: ignoreEmergencyStop
          ? child
          : Selector<RobotProvider, bool?>(
              selector: (context, robotProvider) => robotProvider.isEmergencyStopPressed,
              builder: (context, isEmergencyStopPressed, selectorChild) {
                if (isEmergencyStopPressed == null) {
                  return const Column(
                    children: [
                      Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Padding(
                            padding: EdgeInsets.all(12),
                            child: StatusIndicatorView(),
                          ),
                          Expanded(child: StatusBar()),
                        ],
                      ),
                      Expanded(
                        child: Center(
                            child: Padding(
                          padding: EdgeInsets.symmetric(horizontal: 256),
                          child: Text(
                            "Not-Aus Status konnte nicht abgerufen werden",
                            style: TextStyle(color: RobotColors.primaryText, fontSize: 72),
                            textAlign: TextAlign.center,
                          ),
                        )),
                      ),
                    ],
                  );
                }
                if (isEmergencyStopPressed) {
                  return const EmergencyStopView();
                }
                return TitledView(
                  title: title,
                  showBackButton: showBackButton,
                  onBackButtonPressed: onBackButtonPressed,
                  collapsedTitle: collapsedTitle,
                  child: child,
                );
              },
            ),
    ));
  }
}
