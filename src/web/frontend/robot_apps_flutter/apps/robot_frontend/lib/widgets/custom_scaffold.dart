import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
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
    this.ignoreMissingEmergencyStopData = false,
    this.collapsedTitle = false,
  });

  final Widget child;
  final bool showBackButton;
  final bool inactivityTimerEnabled;
  final VoidCallback? onBackButtonPressed;
  final String title;
  final bool ignoreMissingEmergencyStopData;
  final bool collapsedTitle;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        inactivityTimerEnabled: inactivityTimerEnabled,
        child: Selector<RobotProvider, bool?>(
          selector: (context, robotProvider) => robotProvider.isEmergencyStopPressed,
          builder: (context, isEmergencyStopPressed, selectorChild) {
            if (isEmergencyStopPressed == null && !ignoreMissingEmergencyStopData) {
              return Column(
                children: [
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Expanded(
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(left: 16, top: 8),
                              child: IconButton(
                                padding: const EdgeInsets.all(0),
                                onPressed: () async {
                                  final robotProvider = Provider.of<RobotProvider>(context, listen: false);
                                  robotProvider.blockNavigation();
                                  await Navigator.push(context, MaterialPageRoute<ManualsPage>(builder: (context) => const ManualsPage()));
                                  robotProvider.unblockNavigation();
                                },
                                color: RobotColors.primaryIcon,
                                icon: const Icon(
                                  Icons.info_outline,
                                  size: 64,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                      const Padding(
                        padding: EdgeInsets.all(8),
                        child: ClockView(),
                      ),
                      const Expanded(
                        child: Padding(
                          padding: EdgeInsets.only(top: 8, right: 16),
                          child: StatusIndicatorView(
                            shouldBlockNavigation: true,
                          ),
                        ),
                      ),
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
            if (isEmergencyStopPressed ?? false) {
              Navigator.of(context).popUntil((route) => route is PageRoute);
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
      ),
    );
  }
}
