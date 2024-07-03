import 'dart:async';

import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/menu_page.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class DisinfectionPage extends StatefulWidget {
  const DisinfectionPage({super.key});

  @override
  State<DisinfectionPage> createState() => _DisinfectionPageState();
}

class _DisinfectionPageState extends State<DisinfectionPage> {
  final timeout = const Duration(seconds: 5);
  Timer? timeoutTimer;

  @override
  void initState() {
    super.initState();
    timeoutTimer = Timer(
      timeout,
      () => Navigator.pop(context),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      showBackButton: false,
      child: Column(
        children: [
          Expanded(
            child: GestureDetector(
              onTap: () {
                timeoutTimer?.cancel();
                Navigator.push(
                  context,
                  MaterialPageRoute<MenuPage>(
                    builder: (context) => const MenuPage(),
                  ),
                );
              },
              child: const ColoredBox(
                color: Colors.transparent,
                child: Center(
                  child: Text(
                    'Bitte Hände desinfizieren',
                    style: TextStyle(fontSize: 80),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
