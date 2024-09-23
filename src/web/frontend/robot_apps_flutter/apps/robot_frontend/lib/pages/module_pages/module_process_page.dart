import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/module_process_view.dart';

class ModuleProcessPage extends StatefulWidget {
  const ModuleProcessPage({
    this.requireDisinfection = false,
    super.key,
  });

  final bool requireDisinfection;

  @override
  State<ModuleProcessPage> createState() => _ModuleProcessPageState();
}

class _ModuleProcessPageState extends State<ModuleProcessPage> {
  @override
  Widget build(BuildContext context) {
    Provider.of<InactivityProvider>(context, listen: false).cancelTimer();
    return CustomScaffold(
      showBackButton: false,
      inactivityTimerEnabled: false,
      child: ModuleProcessView(
        requireDesinfection: widget.requireDisinfection,
      ),
    );
  }
}
