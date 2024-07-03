import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/disinfection_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';

import 'package:robot_frontend/widgets/driving_view.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with SingleTickerProviderStateMixin {
  @override
  void initState() {
    super.initState();
    Provider.of<ModulesProvider>(context, listen: false).startModulesUpdateTimer();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        child: Stack(
          children: [
            DrivingView(
              onPressed: () async {
                await Provider.of<RobotProvider>(context, listen: false).blockNavigation();
                if (context.mounted) {
                  await Navigator.push(
                    context,
                    MaterialPageRoute<DisinfectionPage>(
                      builder: (context) => const DisinfectionPage(),
                    ),
                  );
                }
              },
            ),
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
      ),
    );
  }
}
