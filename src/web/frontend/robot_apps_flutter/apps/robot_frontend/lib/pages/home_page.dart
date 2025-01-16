import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/auth_page.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/pages/module_pages/module_process_page.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/disinfection_module_empty_view.dart';

import 'package:robot_frontend/widgets/driving_view.dart';
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
    if (moduleProvider.isInSubmoduleProcess) {
      return;
    }

    moduleProvider.isInSubmoduleProcess = true;
    await Navigator.push(
      context,
      MaterialPageRoute<ModuleProcessPage>(
        builder: (context) => const ModuleProcessPage(
          requireDisinfection: true,
        ),
      ),
    );
    moduleProvider.isInSubmoduleProcess = false;
    if (mounted && (ModalRoute.of(context)?.isCurrent ?? false)) {
      await Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
    }
  }

  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicRemainingDisinfectionsUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicRemainingDisinfectionsUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!isModuleUpdateActive) {
        isModuleUpdateActive = true;
        Provider.of<ModuleProvider>(context, listen: false).startSubmodulesUpdateTimer(
          onModuleProcess: startModuleProcess,
        );
      }
    });
    return CustomScaffold(
      collapsedTitle: true,
      inactivityTimerEnabled: false,
      child: Column(
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
            child: Selector<RobotProvider, int?>(
                selector: (context, provider) => provider.remainingDisinfections,
                builder: (context, remainingDisinfections, child) {
                  if (remainingDisinfections == null) {
                    return const Center(
                        child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: 256),
                      child: Text(
                        "Füllstand des Desinfektionsmoduls konnte nicht abgerufen werden.",
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 72),
                        textAlign: TextAlign.center,
                      ),
                    ));
                  } else if (remainingDisinfections <= 0) {
                    return const DisinfectionModuleEmptyView();
                  }
                  return DrivingView(
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
                  );
                }),
          ),
        ],
      ),
    );
  }
}
