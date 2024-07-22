import 'package:flutter/material.dart';
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
    return CustomScaffold(
      showBackButton: false,
      child: ModuleProcessView(
        requireDesinfection: widget.requireDisinfection,
      ),
    );
  }
}
