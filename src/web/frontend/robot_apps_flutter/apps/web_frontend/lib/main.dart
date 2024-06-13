import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';
import 'package:web_frontend/models/provider/robot_provider.dart';
import 'package:web_frontend/pages/fleet_management_page.dart';

void main() {
  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(
          create: (_) => RobotProvider(),
        ),
        ChangeNotifierProvider(
          create: (_) => RMFProvider(),
        ),
      ],
      child: const WebFrontend(),
    ),
  );
}

class WebFrontend extends StatelessWidget {
  const WebFrontend({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.dark(
        useMaterial3: true,
      ),
      home: const FleetManagementPage(),
    );
  }
}
