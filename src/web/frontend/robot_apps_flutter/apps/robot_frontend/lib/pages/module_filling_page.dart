import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

import 'package:robot_frontend/widgets/module_filling_view.dart';

class ModuleFillingPage extends StatelessWidget {
  const ModuleFillingPage({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(title: 'Module auffüllen/entleeren', child: ModuleFillingView());
  }
}
