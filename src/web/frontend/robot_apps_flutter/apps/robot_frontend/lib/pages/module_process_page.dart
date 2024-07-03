import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/module_process_view.dart';

class ModuleProcessPage extends StatefulWidget {
  const ModuleProcessPage({super.key});

  @override
  State<ModuleProcessPage> createState() => _ModuleProcessPageState();
}

class _ModuleProcessPageState extends State<ModuleProcessPage> {
  @override
  void initState() {
    super.initState();
    Provider.of<ModulesProvider>(context, listen: false).startModulesUpdateTimer();
  }

  @override
  void deactivate() {
    super.deactivate();
    Provider.of<ModulesProvider>(context, listen: false).stopModulesUpdateTimer();
  }

  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      child: ModuleProcessView(),
    );
  }
}
