import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/pages/manual_control_page.dart';
import 'package:web_interface_flutter/pages/manual_move_page.dart';
import 'package:web_interface_flutter/pages/task_creation_page.dart';
import 'package:web_interface_flutter/pages/team_page.dart';
import 'package:web_interface_flutter/widgets/home_page/admin_menu_button.dart';
import 'package:web_interface_flutter/widgets/home_page/dashboard.dart';
import 'package:web_interface_flutter/widgets/home_page/login_button.dart';
import 'package:web_interface_flutter/widgets/is_robot_moving_wrapper.dart';
import 'package:web_interface_flutter/widgets/rounded_button.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  late Future<void> loadData;

  @override
  void initState() {
    super.initState();
    loadData =
        Provider.of<RobotProvider>(context, listen: false).updateProviderData();
    Provider.of<RobotProvider>(context, listen: false)
        .startPeriodicRobotUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicUpdates();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return PopScope(
      canPop: false,
      child: Scaffold(
        backgroundColor: AppColors.grey,
        body: IsRobotMovingWrapper(
          child: Stack(
            fit: StackFit.expand,
            children: [
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 64),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    const Dashboard(),
                    RoundedButton(
                      text: "Schubladensteuerung",
                      color: AppColors.blue,
                      onTap: () => Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => const ManualControlPage(),
                        ),
                      ),
                    ),
                    RoundedButton(
                      text: "Robotersteuerung",
                      color: AppColors.turquoise,
                      onTap: () => Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => const ManualMovePage(),
                        ),
                      ),
                    ),
                    RoundedButton(
                      text: "Neuer Auftrag",
                      color: AppColors.green,
                      onTap: () => Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => const TaskCreationPage(),
                        ),
                      ),
                    ),
                    RoundedButton(
                      text: "Unser Team",
                      color: AppColors.white,
                      textColor: Colors.black,
                      onTap: () => Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => TeamPage(),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              Padding(
                padding: Constants.smallPadding,
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    const LoginButton(),
                    if (Provider.of<RobotProvider>(context, listen: false)
                        .isAdmin) ...[
                      const AdminMenuButton(),
                    ],
                  ],
                ),
              )
            ],
          ),
        ),
      ),
    );
  }
}
