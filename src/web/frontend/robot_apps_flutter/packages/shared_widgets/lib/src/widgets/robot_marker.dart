import 'package:flutter/material.dart';

class RobotMarker extends StatelessWidget {
  const RobotMarker({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        borderRadius: BorderRadius.all(
          Radius.circular(4),
        ),
        color: Colors.blue,
      ),
      child: const Icon(
        Icons.arrow_forward,
        size: 48,
      ),
    );
  }
}
