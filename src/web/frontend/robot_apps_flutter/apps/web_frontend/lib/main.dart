import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/login_page.dart';

void main() {
  final middlewarePrefix = Uri.base.origin.contains('localhost') ? 'http://localhost:8003' : 'http://${Uri.base.host}:8003';
  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(
          create: (_) => FleetProvider(prefix: middlewarePrefix),
        ),
        ChangeNotifierProvider(
          create: (_) => UserProvider(prefix: middlewarePrefix),
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
      ).copyWith(
        textTheme: GoogleFonts.montserratTextTheme(),
      ),
      home: LoginPage(),
    );
  }
}
