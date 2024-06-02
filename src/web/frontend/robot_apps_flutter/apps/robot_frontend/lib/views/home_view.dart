import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/widgets/clock_view.dart';

import 'package:robot_frontend/widgets/driving_view.dart';
import 'package:robot_frontend/widgets/stopped_view.dart';

class HomeView extends StatefulWidget {
  const HomeView({super.key});

  @override
  State<HomeView> createState() => _HomeViewState();
}

class _HomeViewState extends State<HomeView> with SingleTickerProviderStateMixin {
  bool isPressed = false;

  late final AnimationController _controller = AnimationController(
    duration: const Duration(milliseconds: 8000),
    vsync: this,
  );
  late final Animation<Offset> _offsetAnimation = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0, -1.0),
  ).animate(
    CurvedAnimation(
      parent: _controller,
      curve: Curves.easeInOut,
    ),
  );

  late final Animation<Offset> _offsetAnimation2 = Tween<Offset>(
    begin: const Offset(0, 1.0),
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
      body: Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topCenter,
            end: Alignment.bottomCenter,
            colors: [Color(0xFF00DB9E), Color(0xFF8F44F2)],
          ),
        ),
        child: Stack(
          children: [
            SlideTransition(
              position: _offsetAnimation2,
              child: StoppedView(
                onContinue: () async {
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
            Padding(
              padding: const EdgeInsets.only(left: 24, top: 12),
              child: ClockView(),
            ),
            Padding(
              padding: const EdgeInsets.only(right: 24, top: 12),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.end,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Icon(
                    Icons.icecream,
                    size: 48,
                  ),
                  SizedBox(width: 8),
                  Icon(
                    Icons.link,
                    size: 48,
                  ),
                  SizedBox(width: 8),
                  RotatedBox(
                    quarterTurns: 1,
                    child: Icon(
                      Icons.battery_5_bar,
                      size: 48,
                    ),
                  ),
                  SizedBox(width: 8),
                  Container(
                    margin: const EdgeInsets.all(4),
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: Color(0xFF00FF00),
                      border: Border.all(
                        color: Colors.white,
                        width: 4,
                      ),
                    ),
                    width: 38,
                    height: 38,
                  )
                ],
              ),
            )
          ],
        ),
      ),
    );
  }
}
