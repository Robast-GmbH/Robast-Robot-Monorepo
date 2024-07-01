import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';

import 'package:robot_frontend/widgets/driving_view.dart';
import 'package:robot_frontend/widgets/module_process_view.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with SingleTickerProviderStateMixin {
  late final AnimationController _controller = AnimationController(
    duration: const Duration(milliseconds: 800),
    vsync: this,
  );
  late final Animation<Offset> _offsetAnimation = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0, -1),
  ).animate(
    CurvedAnimation(
      parent: _controller,
      curve: Curves.easeInOut,
    ),
  );

  late final Animation<Offset> _offsetAnimation2 = Tween<Offset>(
    begin: const Offset(0, 1),
    end: Offset.zero,
  ).animate(
    CurvedAnimation(
      parent: _controller,
      curve: Curves.easeInOut,
    ),
  );

  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicModulesUpdate();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicIsNavigationBlockedUpdate(onIsNavigationBlockedUpdate);
  }

  void onIsNavigationBlockedUpdate({bool isBlocked = false}) {
    if (_controller.isAnimating) {
      return;
    }
    if (isBlocked) {
      _controller.forward();
    } else {
      _controller.reverse();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Selector<RobotProvider, ModuleProcess?>(
        selector: (_, provider) => provider.moduleProcess,
        builder: (context, moduleProcess, child) {
          return BackgroundView(
            child: Stack(
              children: [
                if (moduleProcess?.state != null && moduleProcess?.state != ModuleProcessState.finished) ...[
                  const ModuleProcessView(),
                ] else ...[
                  SlideTransition(
                    position: _offsetAnimation2,
                    child: DisinfectionView(
                      onTimeout: () async {
                        await Provider.of<RobotProvider>(context, listen: false).unblockNavigation();
                        await _controller.reverse();
                      },
                    ),
                  ),
                  SlideTransition(
                    position: _offsetAnimation,
                    child: DrivingView(
                      onPressed: () async {
                        await Provider.of<RobotProvider>(context, listen: false).blockNavigation();
                        await _controller.forward();
                      },
                    ),
                  ),
                ],
                const Padding(
                  padding: EdgeInsets.only(left: 24, top: 12),
                  child: ClockView(),
                ),
                const Padding(
                  padding: EdgeInsets.only(right: 24, top: 12),
                  child: StatusIndicatorView(),
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}
