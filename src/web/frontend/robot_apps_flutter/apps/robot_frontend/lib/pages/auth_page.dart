import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/menu_page.dart';
import 'package:robot_frontend/widgets/auth_view.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';

class AuthPage extends StatefulWidget {
  const AuthPage({super.key});

  @override
  State<AuthPage> createState() => _AuthPageState();
}

class _AuthPageState extends State<AuthPage> {
  final timeout = const Duration(seconds: 5);
  Timer? timeoutTimer;
  Timer? userSessionUpdateTimer;
  User? authenticatedUser;

  void startTimeoutTimer() {
    timeoutTimer = Timer(
      timeout,
      () {
        Provider.of<RobotProvider>(context, listen: false).unblockNavigation();
        Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
        Navigator.pop(context);
      },
    );
  }

  void startUserSessionUpdateTimer() {
    userSessionUpdateTimer = Timer.periodic(
      const Duration(milliseconds: 300),
      (timer) async {
        authenticatedUser = await Provider.of<UserProvider>(context, listen: false).getUserSession(robotName: 'rb_theron');
        if (authenticatedUser != null) {
          userSessionUpdateTimer?.cancel();
          timeoutTimer?.cancel();
          startTimeoutTimer();
        }
        setState(() {});
      },
    );
  }

  @override
  void initState() {
    super.initState();
    startTimeoutTimer();
    startUserSessionUpdateTimer();
  }

  @override
  void dispose() {
    timeoutTimer?.cancel();
    userSessionUpdateTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        child: authenticatedUser == null
            ? const SizedBox.expand(
                child: Center(
                  child: AuthView(
                    requestedUserIDs: [],
                    requestedUserGroups: [],
                  ),
                ),
              )
            : DisinfectionView(
                onDisinfection: () {
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute<MenuPage>(
                      builder: (context) => const MenuPage(),
                    ),
                  );
                },
              ),
      ),
    );
  }
}
