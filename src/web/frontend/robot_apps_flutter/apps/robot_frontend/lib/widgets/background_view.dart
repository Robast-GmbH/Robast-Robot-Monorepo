import 'package:flutter/material.dart';

class BackgroundView extends StatelessWidget {
  const BackgroundView({super.key, required this.child});
  final Widget child;
  @override
  Widget build(BuildContext context) {
    return Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topCenter,
            end: Alignment.bottomCenter,
            colors: [Color(0xFF00DB9E), Color(0xFF8F44F2)],
          ),
        ),
        child: child);
  }
}
