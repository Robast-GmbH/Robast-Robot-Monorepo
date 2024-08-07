import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/pages/config_page.dart';

void main() {
  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(
          create: (_) => FleetProvider(),
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
      home: const ConfigPage(
        autoClose: true,
      ),
    );
  }
}
