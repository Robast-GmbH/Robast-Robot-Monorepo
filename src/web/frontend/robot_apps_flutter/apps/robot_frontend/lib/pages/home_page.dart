import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/disinfection_page.dart';
import 'package:robot_frontend/pages/module_process_page.dart';
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
  bool isInModuleProcess = false;

  @override
  void initState() {
    super.initState();
    Provider.of<ModuleProvider>(context, listen: false).startModulesUpdateTimer(onModuleProcess: startModuleProcess);
  }

  @override
  void deactivate() {
    Provider.of<ModuleProvider>(context, listen: false).stopModulesUpdateTimer();
    super.deactivate();
  }

  Future<void> startModuleProcess() async {
    if (!context.mounted) {
      return;
    }
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    if (moduleProvider.isInModuleProcess) {
      return;
    }

    moduleProvider.isInModuleProcess = true;
    await Navigator.push(
      context,
      MaterialPageRoute<ModuleProcessPage>(
        builder: (context) => const ModuleProcessPage(),
      ),
    );
    moduleProvider.isInModuleProcess = false;
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
