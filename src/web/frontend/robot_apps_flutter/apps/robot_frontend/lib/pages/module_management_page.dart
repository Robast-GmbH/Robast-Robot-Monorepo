import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/modules_overview.dart';

class ModuleManagementPage extends StatelessWidget {
  const ModuleManagementPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Module verwalten',
      child: Selector<ModuleProvider, List<Submodule>>(
        selector: (context, provider) => provider.submodules,
        builder: (context, submodules, child) {
          return ModulesOverview(
            submodules: submodules,
            displayReservations: true,
          );
        },
      ),
    );
  }
}
