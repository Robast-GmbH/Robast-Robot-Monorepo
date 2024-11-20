import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/menu_page.dart';
import 'package:robot_frontend/widgets/auth_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/disinfection_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class AuthPage extends StatefulWidget {
  const AuthPage({super.key});

  @override
  State<AuthPage> createState() => _AuthPageState();
}

class _AuthPageState extends State<AuthPage> {
  bool authCompleted = false;
  Timer? timeoutTimer;

  void onTimeout() {
    Provider.of<RobotProvider>(context, listen: false).unblockNavigation();
    Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
    Navigator.pop(context);
  }

  void startTimeoutTimer() {
    timeoutTimer = Timer(const Duration(seconds: 5), onTimeout);
  }

  @override
  void dispose() {
    timeoutTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      inactivityTimerEnabled: false,
      collapsedTitle: true,
      child: !authCompleted
          ? SizedBox.expand(
              child: Center(
                child: AuthView(
                  requiredUserIDs: const [],
                  requiredUserGroups: User.availableUserGroups(),
                  onAuthCompleted: ({required bool wasSuccessful}) {
                    if (wasSuccessful) {
                      authCompleted = true;
                      setState(() {});
                    }
                    startTimeoutTimer();
                  },
                  onRetry: () {
                    timeoutTimer?.cancel();
                  },
                ),
              ),
            )
          : DisinfectionView(
              onDisinfection: () {
                if (context.mounted) {
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute<MenuPage>(
                      builder: (context) => const MenuPage(),
                    ),
                  );
                }
              },
            ),
    );
  }
}
