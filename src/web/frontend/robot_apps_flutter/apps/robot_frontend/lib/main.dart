import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/config_page.dart';

class MyCustomScrollBehavior extends MaterialScrollBehavior {
  // Override behavior methods and getters like dragDevices
  @override
  Set<PointerDeviceKind> get dragDevices => {
        PointerDeviceKind.touch,
        PointerDeviceKind.mouse,
        // etc.
      };
}

void main() {
  runApp(
    ChangeNotifierProvider(
      create: (_) => RobotProvider(),
      child: ChangeNotifierProvider(
        create: (_) => ModulesProvider(),
        child: const RobotFrontend(),
      ),
    ),
  );
}

class RobotFrontend extends StatelessWidget {
  const RobotFrontend({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      scrollBehavior: MyCustomScrollBehavior(),
      theme: ThemeData.dark(
        useMaterial3: true,
      ),
      home: const ConfigPage(
        autoClose: true,
      ),
    );
  }
}
