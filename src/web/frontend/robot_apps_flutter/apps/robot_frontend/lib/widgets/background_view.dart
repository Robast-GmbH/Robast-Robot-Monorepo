import 'package:flutter/material.dart';

class BackgroundView extends StatelessWidget {
  const BackgroundView({required this.child, super.key});
  final Widget child;
  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [Color.fromARGB(255, 99, 201, 226), Color.fromARGB(255, 37, 173, 207)],
        ),
      ),
      child: child,
    );
  }
}
