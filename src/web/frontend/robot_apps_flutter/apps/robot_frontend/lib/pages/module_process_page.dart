import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/module_process_view.dart';

class ModuleProcessPage extends StatefulWidget {
  const ModuleProcessPage({super.key});

  @override
  State<ModuleProcessPage> createState() => _ModuleProcessPageState();
}

class _ModuleProcessPageState extends State<ModuleProcessPage> {
  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      showBackButton: false,
      child: ModuleProcessView(),
    );
  }
}
