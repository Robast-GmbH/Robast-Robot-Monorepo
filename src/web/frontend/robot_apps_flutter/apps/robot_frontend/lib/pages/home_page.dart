import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/auth_page.dart';
import 'package:robot_frontend/pages/module_process_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';

import 'package:robot_frontend/widgets/driving_view.dart';
import 'package:robot_frontend/widgets/status_bar.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with SingleTickerProviderStateMixin {
  bool isInModuleProcess = false;
  bool isModuleUpdateActive = false;

  Future<void> startModuleProcess() async {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    if (moduleProvider.isInModuleProcess) {
      return;
    }

    moduleProvider.isInModuleProcess = true;
    await Navigator.push(
      context,
      MaterialPageRoute<ModuleProcessPage>(
        builder: (context) => const ModuleProcessPage(
          requireDisinfection: true,
        ),
      ),
    );
    moduleProvider.isInModuleProcess = false;
    if (mounted && (ModalRoute.of(context)?.isCurrent ?? false)) {
      await Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
    }
  }

  @override
  Widget build(BuildContext context) {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!isModuleUpdateActive) {
        isModuleUpdateActive = true;
        Provider.of<ModuleProvider>(context, listen: false).startModulesUpdateTimer(onModuleProcess: startModuleProcess);
      }
    });
    return Scaffold(
      body: BackgroundView(
        child: Column(
          children: [
            const Row(
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
              child: DrivingView(
                onPressed: () async {
                  await Provider.of<RobotProvider>(context, listen: false).blockNavigation();
                  if (context.mounted) {
                    await Navigator.push(
                      context,
                      MaterialPageRoute<AuthPage>(
                        builder: (context) => const AuthPage(),
                      ),
                    );
                  }
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}
